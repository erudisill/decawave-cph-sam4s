/*
 * i2c.c
 *
 *  Created on: Sep 30, 2015
 *      Author: jcobb
 */

#include "twi.h"
#include "i2c.h"


uint8_t rxBuffer[I2C_BUFFER_LENGTH] = {0};
uint8_t rxBufferIndex = 0;
uint8_t rxBufferLength = 0;

uint8_t txAddress = 0;
uint8_t txBuffer[I2C_BUFFER_LENGTH] = {0};
uint8_t txBufferLength = 0;

uint8_t svcBuffer[I2C_BUFFER_LENGTH] = {0};
uint8_t svcBufferIndex = 0;
uint8_t svcBufferLength = 0;

i2c_status_t i2c_status = UNINITIALIZED;

uint32_t twi_clock = TWI_CLOCK;

Twi *twi = NULL;

void i2c_init(Twi * _twi)
{
//	twi = _twi;
	twi = TWI1;
	i2c_status = UNINITIALIZED;
	twi_clock = TWI_CLOCK;

	rxBufferIndex = 0;
	rxBufferLength = 0;

	rxBufferIndex = 0;
	rxBufferLength = 0;

	svcBufferIndex = 0;
	svcBufferLength = 0;
}

static inline bool TWI_FailedAcknowledge(Twi *pTwi) {
	return pTwi->TWI_SR & TWI_SR_NACK;
}

static inline bool TWI_WaitTransferComplete(Twi *_twi, uint32_t _timeout) {
	uint32_t _status_reg = 0;
	while ((_status_reg & TWI_SR_TXCOMP) != TWI_SR_TXCOMP) {
		_status_reg = TWI_GetStatus(_twi);

		if (_status_reg & TWI_SR_NACK)
			return false;

		if (--_timeout == 0)
			return false;
	}
	return true;
}


static inline bool TWI_WaitByteSent(Twi *_twi, uint32_t _timeout) {
	uint32_t _status_reg = 0;
	while ((_status_reg & TWI_SR_TXRDY) != TWI_SR_TXRDY) {
		_status_reg = TWI_GetStatus(_twi);

		if (_status_reg & TWI_SR_NACK)
			return false;

		if (--_timeout == 0)
			return false;
	}

	return true;
}

static inline bool TWI_WaitByteReceived(Twi *_twi, uint32_t _timeout) {
	uint32_t _status_reg = 0;
	while ((_status_reg & TWI_SR_RXRDY) != TWI_SR_RXRDY) {
		_status_reg = TWI_GetStatus(_twi);

		if (_status_reg & TWI_SR_NACK)
			return false;

		if (--_timeout == 0)
			return false;
	}

	return true;
}

static inline bool TWI_STATUS_SVREAD(uint32_t status) {
	return (status & TWI_SR_SVREAD) == TWI_SR_SVREAD;
}

static inline bool TWI_STATUS_SVACC(uint32_t status) {
	return (status & TWI_SR_SVACC) == TWI_SR_SVACC;
}

static inline bool TWI_STATUS_GACC(uint32_t status) {
	return (status & TWI_SR_GACC) == TWI_SR_GACC;
}

static inline bool TWI_STATUS_EOSACC(uint32_t status) {
	return (status & TWI_SR_EOSACC) == TWI_SR_EOSACC;
}

static inline bool TWI_STATUS_NACK(uint32_t status) {
	return (status & TWI_SR_NACK) == TWI_SR_NACK;
}

void i2c_begin(void)
{

	// Disable PDC channel
	twi->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

	TWI_ConfigureMaster(twi, twi_clock, VARIANT_MCK);
	i2c_status = MASTER_IDLE;
}

void i2c_set_clock(uint32_t frequency)
{
	twi_clock = frequency;
	TWI_SetClock(twi, twi_clock, VARIANT_MCK);
}

uint8_t i2c_request_from(uint8_t address, uint8_t quantity, uint32_t iaddress, uint8_t isize, uint8_t send_stop)
{
	if (quantity > I2C_BUFFER_LENGTH)
		quantity = I2C_BUFFER_LENGTH;

	// perform blocking read into buffer
	int readed = 0;
	TWI_StartRead(twi, address, iaddress, isize);
	do {
		// Stop condition must be set during the reception of last byte
		if (readed + 1 == quantity)
			TWI_SendSTOPCondition( twi);

		if (TWI_WaitByteReceived(twi, RECV_TIMEOUT))
			rxBuffer[readed++] = TWI_ReadByte(twi);
		else
			break;
	} while (readed < quantity);
	TWI_WaitTransferComplete(twi, RECV_TIMEOUT);

	// set rx buffer iterator vars
	rxBufferIndex = 0;
	rxBufferLength = readed;

	return readed;

}



void i2c_begin_transmission(uint8_t address)
{
	i2c_status = MASTER_SEND;

	// save address of target and empty buffer
	txAddress = address;
	txBufferLength = 0;
}

uint8_t i2c_end_transmission(uint8_t send_stop)
{
	uint8_t error = 0;
	// transmit buffer (blocking)
	TWI_StartWrite(twi, txAddress, 0, 0, txBuffer[0]);
	if (!TWI_WaitByteSent(twi, XMIT_TIMEOUT))
		error = 2;	// error, got NACK on address transmit

	if (error == 0) {
		uint16_t sent = 1;
		while (sent < txBufferLength) {
			TWI_WriteByte(twi, txBuffer[sent++]);
			if (!TWI_WaitByteSent(twi, XMIT_TIMEOUT))
				error = 3;	// error, got NACK during data transmmit
		}
	}

	if (error == 0) {
		TWI_Stop(twi);
		if (!TWI_WaitTransferComplete(twi, XMIT_TIMEOUT))
			error = 4;	// error, finishing up
	}

	txBufferLength = 0;		// empty buffer
	i2c_status = MASTER_IDLE;
	return error;
}


uint8_t i2c_write(uint8_t *data, uint8_t size)
{
	if (i2c_status == MASTER_SEND) {
		for (size_t i = 0; i < size; ++i) {
			if (txBufferLength >= I2C_BUFFER_LENGTH)
				return i;
			txBuffer[txBufferLength++] = data[i];
		}
	} else {
		for (size_t i = 0; i < size; ++i) {
			if (svcBufferLength >= I2C_BUFFER_LENGTH)
				return i;
			svcBuffer[svcBufferLength++] = data[i];
		}
	}
	return size;
}

uint8_t i2c_write_byte(uint8_t data)
{
	if (i2c_status == MASTER_SEND) {
		if (txBufferLength >= I2C_BUFFER_LENGTH)
			return 0;
		txBuffer[txBufferLength++] = data;
		return 1;
	} else {
		if (svcBufferLength >= I2C_BUFFER_LENGTH)
			return 0;
		svcBuffer[svcBufferLength++] = data;
		return 1;
	}
}

uint8_t i2c_read(void)
{
	if (rxBufferIndex < rxBufferLength)
		return rxBuffer[rxBufferIndex++];
	return -1;
}


uint8_t i2c_available(void)
{
	return rxBufferLength - rxBufferIndex;
}

uint8_t i2c_peek(void)
{
	if (rxBufferIndex < rxBufferLength)
		return rxBuffer[rxBufferIndex];
	return -1;
}
