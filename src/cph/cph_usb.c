/*
 * cph_usb.c
 *
 *  Created on: Apr 14, 2016
 *      Author: jcobb
 */


#include <cph.h>
#include <cph_usb.h>
#include <stdio_usb.h>

static cph_usb_rx_notify_cb_t rx_notify_cb;

volatile bool cph_usb_rx_ready = false;

void cph_usb_rx_notify(void)
{
	cph_usb_rx_ready = true;
	if (rx_notify_cb)
		rx_notify_cb();
}

void cph_usb_init(void)
{
	rx_notify_cb = 0;
//	cph_stdio_init();
	stdio_usb_init();
}

bool cph_usb_data_ready(void)
{
	if(cph_usb_rx_ready) {
		cph_usb_rx_ready = false;
		return true;
	} else {
		return false;
	}

}

void cph_usb_data_read(uint8_t *data)
{
	stdio_usb_getchar(NULL, data);
}

void cph_usb_set_rx_notify_cb(cph_usb_rx_notify_cb_t cb)
{
	rx_notify_cb = cb;
}
