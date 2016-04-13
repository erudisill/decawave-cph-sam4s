/*
 * imu.c
 *
 *  Created on: Oct 2, 2015
 *      Author: jcobb
 */
#include <string.h>
#include "cph_millis.h"
#include "board.h"
#include "i2c.h"
#include "i2c_driver.h"
#include "imu.h"

uint8_t imu_address = IMU_ADDRESS;
static uint8_t imu_buffer[I2C_BUFFER_LENGTH] = {0};

static volatile uint8_t irq_set = 0x00;


void imu_process_interrupt(uint32_t id, uint32_t mask)
{
	irq_set = 0x01;
}

bool imu_irq_ready(void)
{
	return irq_set;
}

void imu_irq_reset(void)
{
	irq_set = 0x00;
}

void imu_init_default(void)
{
	imu_init();
	imu_set_rate(7);
	cph_millis_delay(10);
	imu_set_dlpf_mode(MPU6050_DLPF_BW_5);
}

void imu_init(void)
{

	pmc_enable_periph_clk(IMU_TWI_ID);
	i2c_init(TWI0);
	i2c_begin();

	memset(imu_buffer, 0, sizeof(imu_buffer));

//	imu_reset();
//	cph_millis_delay(10);

	imu_set_clock_source(MPU6050_CLOCK_PLL_XGYRO);
	cph_millis_delay(10);

	imu_set_full_scale_gyro_range(MPU6050_GYRO_FS_250);
	cph_millis_delay(10);

	imu_set_full_scale_accel_range(MPU6050_ACCEL_FS_2);
	cph_millis_delay(10);

	imu_set_sleep_enabled(false);
	cph_millis_delay(10);

	imu_set_int_enabled(IMU_INTERRUPT_ENABLE);
	cph_millis_delay(10);

}

void imu_reset(void)
{
	bool status = writeBit(imu_address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET_BIT, true);

	if(status == true)
		printf("imu_reset: success\r\n");
	else
		printf("imu_reset: failed\r\n");
}

bool imu_test_connection(void)
{
	return imu_get_device_id() == 0x34;
}

uint8_t imu_get_int_dataready_status(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBits(imu_address, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_DATA_RDY_BIT, 1, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];

}

void imu_set_clock_source(uint8_t source)
{
	bool status = writeBits(imu_address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

		if(status == true)
		{
			printf("set_clock_source: success\r\n");
		}
		else
			printf("set_clock_source: failed\r\n");
}

void imu_set_sleep_enabled(bool enabled)
{
	bool status = writeBit(imu_address, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);

	if(status == true)
	{
		printf("set_sleep_enabled: success\r\n");
	}
	else
		printf("set_sleep_enabled: failed\r\n");

}

uint8_t imu_get_device_id(void)
{
//	uint8_t imu_buffer[IMU_BUFFER_LEN];
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBits(imu_address, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	uint8_t device_id = imu_buffer[0];

	printf("device_id: %02X\r\n", device_id);

	return device_id;
}

void imu_set_full_scale_gyro_range(uint8_t range)
{
	bool status = writeBits(imu_address, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);

	if(status == true)
	{
		printf("set_full_scale_gyro_range: success\r\n");
	}
	else
		printf("set_full_scale_gyro_range: failed\r\n");
}

void imu_set_full_scale_accel_range(uint8_t range)
{
	bool status = writeBits(imu_address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);

	if(status == true)
	{
		printf("set_full_scale_accel_range: success\r\n");
	}
	else
		printf("set_full_scale_accel_range: failed\r\n");
}

void imu_getmotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz)
{
//	uint8_t imu_buffer[I2C_BUFFER_LENGTH];
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBytes(imu_address, MPU6050_RA_ACCEL_XOUT_H, 14, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
    *ax = (((int32_t)imu_buffer[0]) << 8) | imu_buffer[1];
    *ay = (((int32_t)imu_buffer[2]) << 8) | imu_buffer[3];
    *az = (((int32_t)imu_buffer[4]) << 8) | imu_buffer[5];
    *gx = (((int32_t)imu_buffer[8]) << 8) | imu_buffer[9];
    *gy = (((int32_t)imu_buffer[10]) << 8) | imu_buffer[11];
    *gz = (((int32_t)imu_buffer[12]) << 8) | imu_buffer[13];
}

void imu_getmotion9(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz)
{
//	uint8_t imu_buffer[I2C_BUFFER_LENGTH];
	memset(imu_buffer, 0, sizeof(imu_buffer));

	// get accel and gyro
	imu_getmotion6(ax, ay, az, gx, gy, gz);

	// set i2c bypass to access magnetometer
	writeByte(imu_address, MPU6050_RA_INT_PIN_CFG, 0x02);
	cph_millis_delay(10);

	// enable magnetometer
	writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01);
	cph_millis_delay(10);

	// read magnetometer
	readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
	*mx = (((int16_t)imu_buffer[1]) << 8) | imu_buffer[0];
	*my = (((int16_t)imu_buffer[3]) << 8) | imu_buffer[2];
	*mz = (((int16_t)imu_buffer[5]) << 8) | imu_buffer[4];

}

void imu_get_rotation(int16_t *x, int16_t *y, int16_t *z)
{
//	uint8_t imu_buffer[I2C_BUFFER_LENGTH];
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBytes(imu_address, MPU6050_RA_GYRO_XOUT_H, 6, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
    *x = (((int16_t)imu_buffer[0]) << 8) | imu_buffer[1];
    *y = (((int16_t)imu_buffer[2]) << 8) | imu_buffer[3];
    *z = (((int16_t)imu_buffer[4]) << 8) | imu_buffer[5];
}

void imu_get_acceleration(int16_t *x, int16_t *y, int16_t *z)
{
//	uint8_t imu_buffer[I2C_BUFFER_LENGTH];
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBytes(imu_address, MPU6050_RA_ACCEL_XOUT_H, 6, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
    *x = (((int16_t)imu_buffer[0]) << 8) | imu_buffer[1];
    *y = (((int16_t)imu_buffer[2]) << 8) | imu_buffer[3];
    *z = (((int16_t)imu_buffer[4]) << 8) | imu_buffer[5];
}

void imu_get_mag(int16_t *x, int16_t *y, int16_t *z)
{
//	uint8_t imu_buffer[I2C_BUFFER_LENGTH];
	memset(imu_buffer, 0, sizeof(imu_buffer));

	// set i2c bypass to access magnetometer
	writeByte(imu_address, MPU6050_RA_INT_PIN_CFG, 0x02);
	cph_millis_delay(10);

	// enable magnetometer
	writeByte(MPU9150_RA_MAG_ADDRESS, 0x0A, 0x01);
	cph_millis_delay(10);

	// read magnetometer
	readBytes(MPU9150_RA_MAG_ADDRESS, MPU9150_RA_MAG_XOUT_L, 6, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
	*x = (((int16_t)imu_buffer[1]) << 8) | imu_buffer[0];
	*y = (((int16_t)imu_buffer[3]) << 8) | imu_buffer[2];
	*z = (((int16_t)imu_buffer[5]) << 8) | imu_buffer[4];
}

uint8_t imu_get_dlpf_mode()
{
//	uint8_t imu_buffer[IMU_BUFFER_LEN];
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBits(imu_address, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];
}

void imu_set_dlpf_mode(uint8_t mode)
{
	writeBits(imu_address, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

void imu_set_int_enabled(uint8_t enabled)
{
	writeByte(imu_address, MPU6050_RA_INT_ENABLE, enabled);
}

void imu_set_rate(uint8_t rate)
{
	writeByte(imu_address, MPU6050_RA_SMPLRT_DIV, rate);
}

