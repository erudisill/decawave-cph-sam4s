/*
 * i2c.h
 *
 *  Created on: Sep 30, 2015
 *      Author: jcobb
 */

#ifndef I2C_H_
#define I2C_H_

#include "compiler.h"
/** Frequency of the board main oscillator */
#define VARIANT_MAINOSC		12000000

/** Master clock frequency */
#define VARIANT_MCK			84000000

//#define TWI_CLOCK  	100000
#define TWI_CLOCK   400000

#define I2C_BUFFER_LENGTH		32

//#define I2C_DEBUG


void i2c_init(Twi * twi);
void i2c_begin(void);
void i2c_set_clock(uint32_t frequency);
void i2c_begin_transmission(uint8_t address);
uint8_t i2c_end_transmission(uint8_t send_stop);
uint8_t i2c_request_from(uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize, uint8_t send_stop);
uint8_t i2c_write(uint8_t *data, uint8_t size);
uint8_t i2c_write_byte(uint8_t data);
uint8_t i2c_read(void);
uint8_t i2c_available(void);
uint8_t i2c_peek(void);


uint8_t rxBuffer[I2C_BUFFER_LENGTH];
uint8_t rxBufferIndex;
uint8_t rxBufferLength;

uint8_t txAddress;
uint8_t txBuffer[I2C_BUFFER_LENGTH];
uint8_t txBufferLength;

uint8_t svcBuffer[I2C_BUFFER_LENGTH];
uint8_t svcBufferIndex;
uint8_t svcBufferLength;

Twi *twi;

static const uint32_t RECV_TIMEOUT = 100000;
static const uint32_t XMIT_TIMEOUT = 100000;

typedef enum i2c_status_codes {
	UNINITIALIZED,
	MASTER_IDLE,
	MASTER_SEND,
	MASTER_RECV,
	SLAVE_IDLE,
	SLAVE_RECV,
	SLAVE_SEND
}i2c_status_t;

i2c_status_t i2c_status;


uint32_t twi_clock;




#endif /* I2C_H_ */
