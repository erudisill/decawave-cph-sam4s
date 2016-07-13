/*
 * cph_stdio.c
 *
 *  Created on: Sep 23, 2015
 *      Author: ericrudisill
 */

#include <cph_stdio.h>
#include <cph_usb.h>
#include <stdio_usb.h>

static cph_stdio_dataready_cb_t dataready_cb;

void cph_uart_init(void);

void cph_stdio_init(void) {

#if defined(STDIO_ROUTE_UART)
	cph_uart_init();
#elif defined(STDIO_ROUTE_USB)
	cph_usb_init();
#endif

}

void cph_uart_init(void)
{
	const usart_serial_options_t uart_serial_options = { .baudrate = CONF_UART_BAUDRATE, .paritytype = CONF_UART_PARITY, };
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

bool cph_stdio_dataready(void)
{

#if defined(STDIO_ROUTE_UART)
	if (uart_is_rx_ready(CONSOLE_UART)) {
		return true;
	} else {
		return false;
	}
#elif defined(STDIO_ROUTE_USB)
	if(cph_usb_rx_ready) {
		cph_usb_rx_ready = false;
		return true;
	} else {
		return false;
	}
#endif

}

void cph_stdio_readc(uint8_t *data)
{

#if defined(STDIO_ROUTE_UART)
	uart_read(CONSOLE_UART, data);
#elif defined(STDIO_ROUTE_USB)
	stdio_usb_getchar(NULL, data);
#endif
}

void cph_stdio_set_datready_cb(cph_stdio_dataready_cb_t cb) {
	dataready_cb = cb;
#if defined(STDIO_ROUTE_UART)
	// For UART, setup interrupt for RX READY event
#elif defined(STDIO_ROUTE_USB)
	// For USB, just pass the callback on to rx_notify handler
	cph_usb_set_rx_notify_cb(dataready_cb);
#endif
}

int cph_stdio_read_buf(uint8_t *data, int maxlen)
{
#if defined(STDIO_ROUTE_UART)
//#error cph_stdio_read not defined for UART yet
#elif defined(STDIO_ROUTE_USB)
	int count = udi_cdc_get_nb_received_data();
	if (count > maxlen) {
		count = maxlen;
	}

	for (int i=0; i < count; i++) {
		data[i] = udi_cdc_getc();
	}
	return count;
#endif
}






