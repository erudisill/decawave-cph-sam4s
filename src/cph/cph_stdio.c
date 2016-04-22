/*
 * cph_stdio.c
 *
 *  Created on: Sep 23, 2015
 *      Author: ericrudisill
 */

#include <cph_stdio.h>
#include <cph_usb.h>
#include <stdio_usb.h>


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


