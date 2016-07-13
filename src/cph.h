/*
 * cph.h
 *
 *  Created on: Sep 23, 2015
 *      Author: ericrudisill
 */

#ifndef SRC_CPH_H_
#define SRC_CPH_H_

#define FW_MAJOR				0x01
#define FW_MINOR				0x02

//#define ANCHOR
//#define COORDINATOR
#define TAG
//#define SENDER
//#define LISTENER

//#define STDIO_ROUTE_UART		0x01
#define STDIO_ROUTE_USB			0x01

#define TRACE(...)				printf(__VA_ARGS__)


#include <asf.h>

#include <cph_config.h>
#include <cph_millis.h>
#include <cph_stdio.h>
#include <cph_deca.h>
#include <cph_utils.h>
#include <cph_usb.h>
#include <globals.h>
#include <configure.h>

typedef struct PACKED {
	uint8_t magic[4];
	uint8_t hw_major;
	uint8_t hw_minor;
	uint8_t fw_major;
	uint8_t fw_minor;
	uint16_t panid;
	uint16_t shortid;
	dwt_config_t dwt_config;
	uint8_t mode;
	uint32_t sender_period;
	uint16_t sender_target;
} cph_config_t;


extern cph_config_t * cph_config;

extern int cph_mode;

extern uint16_t cph_coordid;

void cph_board_init(void);



#endif /* SRC_CPH_H_ */
