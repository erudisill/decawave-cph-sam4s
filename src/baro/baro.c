/*
 * baro.c
 *
 *  Created on: Apr 14, 2016
 *      Author: jcobb
 */

#include <string.h>
#include "cph_millis.h"
#include "board.h"
#include "i2c.h"
#include "i2c_driver.h"
#include "baro.h"

uint8_t baro_mode = 0;

uint8_t baro_address = MPL3115A2_ADDRESS;
static uint8_t baro_buffer[I2C_BUFFER_LENGTH] = {0};

static volatile uint8_t irq_set = 0x00;

void baro_init(void)
{

}

bool baro_begin(void)
{
	pmc_enable_periph_clk(BARO_TWI_ID);
	i2c_init(BARO_TWI);
	i2c_begin();

	memset(baro_buffer, 0, sizeof(baro_buffer));

	uint8_t whoami = read8(MPL3115A2_WHOAMI);

	write8(MPL3115A2_CTRL_REG1,
			MPL3115A2_CTRL_REG1_SBYB |
			MPL3115A2_CTRL_REG1_OS128 |
			MPL3115A2_CTRL_REG1_ALT);

	write8(MPL3115A2_PT_DATA_CFG,
		MPL3115A2_PT_DATA_CFG_TDEFE |
		MPL3115A2_PT_DATA_CFG_PDEFE |
		MPL3115A2_PT_DATA_CFG_DREM);

	return true;

}

uint32_t baro_get_pressure(void)
{

	  uint32_t pressure;

	  write8(MPL3115A2_CTRL_REG1,
		 MPL3115A2_CTRL_REG1_SBYB |
		 MPL3115A2_CTRL_REG1_OS128 |
		 MPL3115A2_CTRL_REG1_BAR);

	  uint8_t sta = 0;
	  while (! (sta & MPL3115A2_REGISTER_STATUS_PDR)) {
	    sta = read8(MPL3115A2_REGISTER_STATUS);
	    cph_millis_delay(125);
	  }




//	  Wire.beginTransmission(MPL3115A2_ADDRESS); // start transmission to device
//	  Wire.write(MPL3115A2_REGISTER_PRESSURE_MSB);
//	  Wire.endTransmission(false); // end transmission

	  bool status = writeBit(MPL3115A2_ADDRESS, MPL3115A2_REGISTER_PRESSURE_MSB, MPL3115A2_OUT_P_DELTA_MSB, true);

//	  Wire.requestFrom((uint8_t)MPL3115A2_ADDRESS, (uint8_t)3);// send data n-bytes read
//	  pressure = Wire.read(); // receive DATA
//	  pressure <<= 8;
//	  pressure |= Wire.read(); // receive DATA
//	  pressure <<= 8;
//	  pressure |= Wire.read(); // receive DATA
//	  pressure >>= 4;

	  readBits(MPL3115A2_ADDRESS, 0x01, 0x00, 3, baro_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	  pressure = (((int32_t)baro_buffer[0]) << 8) | (((int32_t)baro_buffer[1]) << 8) | (((int32_t)baro_buffer[2]) << 4);

	  float baro = pressure;
	  baro /= 4.0;
	  return baro;
}

uint32_t baro_get_altitude(void)
{

	return 0;
}

uint32_t baro_get_temperature(void)
{

	return 0;
}

void write8(uint8_t a, uint8_t d)
{

}

uint8_t read8(uint8_t a)
{

}


