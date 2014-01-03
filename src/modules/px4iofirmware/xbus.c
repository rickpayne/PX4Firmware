/**
 * @file xbus.c
 *
 * Serial protocol decoder for the SXRL Multiplex protocol as used by JR in their X-Bus Mode B
 */

#include <nuttx/config.h>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <systemlib/ppm_decode.h>

#include <drivers/drv_hrt.h>

#define DEBUG
#include "px4io.h"
#include "protocol.h"
#include "debug.h"

#define XBUS_FRAME_SIZE		27      // Start byte, 12 chans * 2 bytes, 2 bytes CRC
#define XBUS_INPUT_CHANNELS	12

static int xbus_fd = -1;

static hrt_abstime last_rx_time;
static hrt_abstime last_frame_time;

static uint8_t	frame[XBUS_FRAME_SIZE];

static unsigned partial_frame_count;

unsigned xbus_frame_drops;
unsigned xbus_crc_drops;

static bool xbus_decode(hrt_abstime frame_time, uint16_t *values, uint16_t *num_values, uint16_t max_channels);

int
xbus_init(const char *device)
{
	if (xbus_fd < 0)
		xbus_fd = open(device, O_RDONLY | O_NONBLOCK);

	if (xbus_fd >= 0) {
		struct termios t;

		/* 115200bps, no parity, 1 stop bits */
		tcgetattr(xbus_fd, &t);
		cfsetspeed(&t, 115200);
		tcsetattr(xbus_fd, TCSANOW, &t);

		/* initialise the decoder */
		partial_frame_count = 0;
		last_rx_time = hrt_absolute_time();

		debug("X.Bus: ready");

	} else {
		debug("X.Bus: open failed");
	}

	return xbus_fd;
}

bool
xbus_input(uint16_t *values, uint16_t *num_values, uint16_t *rssi, uint16_t max_channels)
{
	ssize_t		ret;
	hrt_abstime	now;

	/*
	 * The X.bus protocol doesn't provide reliable framing,
	 * so we detect frame boundaries by the inter-frame delay.
	 *
	 * The minimum frame spacing is 10ms; with 25 bytes at 115200bps
	 * frame transmission time is ~2ms.
	 *
	 * We expect to only be called when bytes arrive for processing,
	 * and if an interval of more than 3ms passes between calls,
	 * the first byte we read will be the first byte of a frame.
	 *
	 * In the case where byte(s) are dropped from a frame, this also
	 * provides a degree of protection. Of course, it would be better
	 * if we didn't drop bytes...
	 */
	now = hrt_absolute_time();

	if ((now - last_rx_time) > 3000) {
		if (partial_frame_count > 0) {
			xbus_frame_drops++;
			partial_frame_count = 0;
		}
	}

	/*
	 * Fetch bytes, but no more than we would need to complete
	 * the current frame.
	 */
	ret = read(xbus_fd, &frame[partial_frame_count], XBUS_FRAME_SIZE - partial_frame_count);

	/* if the read failed for any reason, just give up here */
	if (ret < 1)
		return false;

	last_rx_time = now;

	/*
	 * Add bytes to the current frame
	 */
	partial_frame_count += ret;

	/*
	 * If we don't have a full frame, return
	 */
	if (partial_frame_count < XBUS_FRAME_SIZE)
		return false;

	/*
	 * Great, it looks like we might have a frame.  Go ahead and
	 * decode it.
	 */
	partial_frame_count = 0;
        *rssi = 255;
	return xbus_decode(now, values, num_values, max_channels);
}

static uint16_t
xbus_CRC16(uint16_t crc, uint8_t value)
{
	uint8_t i;

	crc = crc ^ (uint16_t)value<<8;
	for (i=0; i<8; i++) {
          if (crc & 0x8000) {
            crc=crc << 1 ^0x1021;
          } else {
            crc = crc << 1;
          }
    }
    return crc;
}


static bool
xbus_decode(hrt_abstime frame_time, uint16_t *values, uint16_t *num_values, uint16_t max_values)
{
	uint16_t crc;

	/* check frame boundary markers to avoid out-of-sync cases */
	if (frame[0] != 0xA1)  {
		xbus_frame_drops++;
		return false;
	}

	crc = 0;
	for (unsigned i = 0; i <= 24; i++) xbus_CRC16(crc, frame[i]);

        if (crc != (frame[25] << 8 | frame[26])) {
          xbus_crc_drops++;
          return false;
        }

	/* we have received something we think is a frame */
	last_frame_time = frame_time;

	unsigned chancount = (max_values > XBUS_INPUT_CHANNELS) ?
			     XBUS_INPUT_CHANNELS : max_values;

	/* use the decoder matrix to extract channel data */
	unsigned channel = 0;
	for (unsigned byte = 1; byte < (1 + (chancount << 1)); byte += 2) {
		/* convert 0-4095 values to 1000-2000 ppm encoding in a very sloppy fashion */
          values[channel++] = ((frame[byte] << 8 | frame[byte + 1]) >> 2) + 977;
	}

	/* note the number of channels decoded */
	*num_values = chancount;

	return true;
}
