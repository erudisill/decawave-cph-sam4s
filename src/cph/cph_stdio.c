/*
 * cph_stdio.c
 *
 *  Created on: Sep 23, 2015
 *      Author: ericrudisill
 */

#include <cph_stdio.h>


static struct usart_module usart_instance;


void cph_stdio_init(void) {
	const usart_serial_options_t uart_serial_options = { .baudrate = CONF_UART_BAUDRATE, .paritytype = CONF_UART_PARITY, };
	stdio_serial_init(CONF_UART, &uart_serial_options);
}


