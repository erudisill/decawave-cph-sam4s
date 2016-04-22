/*
 * imu_api.c
 *
 *  Created on: Apr 22, 2016
 *      Author: jcobb
 */

#include <string.h>
#include "cph_millis.h"
#include "board.h"
#include "i2c.h"
#include "i2c_driver.h"
#include "imu.h"
#include "imu_api.h"

uint8_t imu_address = IMU_ADDRESS;
uint8_t imu_buffer[I2C_BUFFER_LENGTH] = {0};

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

uint8_t imu_get_int_enabled(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readByte(imu_address, MPU9150_RA_INT_ENABLE, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
	return imu_buffer[0];
}


bool imu_get_cycle_bit(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBit(imu_address, MPU9150_RA_PWR_MGMT_1, MPU9150_PWR1_CYCLE_BIT, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
	return imu_buffer[0];
}

bool imu_get_sleep_bit(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBit(imu_address, MPU9150_RA_PWR_MGMT_1, MPU9150_PWR1_SLEEP_BIT, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
	return imu_buffer[0];
}

bool imu_get_standby_bit(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBit(imu_address, MPU9150_RA_PWR_MGMT_1, MPU9150_PWR1_STANDBY_BIT, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
	return imu_buffer[0];
}



uint8_t imu_get_reg(uint8_t reg)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

//	readBytes(imu_address, reg, 8, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	readBits(imu_address, reg, 0, 8, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
	cph_millis_delay(10);
	return imu_buffer[0];

}

bool imu_get_reg_bit(uint8_t reg, uint8_t bit)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBit(imu_address, reg, bit, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
	return imu_buffer[0];
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

	readBits(imu_address, MPU9150_RA_CONFIG, MPU9150_CFG_DLPF_CFG_BIT, MPU9150_CFG_DLPF_CFG_LENGTH, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];
}

uint8_t imu_get_accel_config2(void)
{

	memset(imu_buffer, 0, sizeof(imu_buffer));
	static volatile uint8_t bit0 = 0;
	static volatile uint8_t bit1 = 0;
	static volatile uint8_t bit2 = 0;
	static volatile uint8_t bit3 = 0;
	static volatile uint8_t reg_1d = 0;
	static volatile uint8_t reg_bits = 0;

	TRACE("ACCEL_CONFIG 2 (0x1D) ");
	// accel_fchoice_b
	readBit(imu_address, MPU9150_RA_FF_THR, 3, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
	bit3 = imu_buffer[0];
	memset(imu_buffer, 0, sizeof(imu_buffer));
	TRACE("accel_fchoice_b bit [3] %d\r\n", bit3);


	TRACE("ACCEL_CONFIG 2 (0x1D) ");
	// A_DLPFCFG
	readBit(imu_address, MPU9150_RA_FF_THR, 0, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
	bit0 = imu_buffer[0];
	memset(imu_buffer, 0, sizeof(imu_buffer));

	// A_DLPFCFG
	readBit(imu_address, MPU9150_RA_FF_THR, 1, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
	bit1 = imu_buffer[0];
	memset(imu_buffer, 0, sizeof(imu_buffer));

	// A_DLPFCFG
	readBit(imu_address, MPU9150_RA_FF_THR, 2, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
	bit2 = imu_buffer[0];
	memset(imu_buffer, 0, sizeof(imu_buffer));

	TRACE("A_DLPFCFG bits [2:0] %d %d %d\r\n", bit2, bit1, bit0);

	return 0;
//
//	// read entire byte
//	readByte(imu_address, MPU9150_RA_FF_THR, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
//	reg_1d = imu_buffer[0];
//	memset(imu_buffer, 0, sizeof(imu_buffer));
//
//	// read all bits
//	readBits(imu_address, MPU9150_RA_FF_THR, 0, 8, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
//	reg_bits = 0;
//	memset(imu_buffer, 0, sizeof(imu_buffer));
//
//	return imu_buffer[0];
}

void imu_set_dlpf_mode(uint8_t mode)
{
	volatile bool status = false;

	status = writeBits(imu_address, MPU9150_RA_CONFIG, MPU9150_CFG_DLPF_CFG_BIT, MPU9150_CFG_DLPF_CFG_LENGTH, mode);
}

void imu_set_int_enabled(uint8_t enabled)
{
	volatile bool status = false;

	status = writeByte(imu_address, MPU6050_RA_INT_ENABLE, enabled);

	if(status == true)
	{
		printf("imu_set_int_enabled: success\r\n");
	}
	else
		printf("imu_set_int_enabled: failed\r\n");
}

void imu_set_rate(uint8_t rate)
{
	volatile bool status = false;

	status = writeByte(imu_address, MPU6050_RA_SMPLRT_DIV, rate);
}

void imu_set_motion_detection_threshold(uint8_t threshold)
{
	volatile bool status = false;

	status = writeByte(imu_address, MPU6050_RA_MOT_THR, threshold);
}

void imu_set_zero_motion_detection_threshold(uint8_t threshold)
{
	volatile bool status = false;

	status = writeByte(imu_address, MPU6050_RA_ZRMOT_THR, threshold);
}

void imu_set_motion_detection_duration(uint8_t threshold)
{
	volatile bool status = false;

	status = writeByte(imu_address, MPU6050_RA_MOT_DUR, threshold);
}

void imu_set_zero_motion_detection_duration(uint8_t threshold)
{
	volatile bool status = false;

	status = writeByte(imu_address, MPU6050_RA_ZRMOT_DUR, threshold);
}

void imu_set_standby_x_accel_enabled(bool enabled)
{
	bool status = writeBit(imu_address, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_XA_BIT, enabled);

	if(status == true)
		printf("imu_set_standby_x_accel_enabled: success\r\n");
	else
		printf("imu_set_standby_x_accel_enabled: failed\r\n");
}

void imu_set_standby_y_accel_enabled(bool enabled)
{
	bool status = writeBit(imu_address, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_YA_BIT, enabled);

	if(status == true)
		printf("imu_set_standby_y_accel_enabled: success\r\n");
	else
		printf("imu_set_standby_y_accel_enabled: failed\r\n");
}

void imu_set_standby_z_accel_enabled(bool enabled)
{
	bool status = writeBit(imu_address, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_STBY_ZA_BIT, enabled);

	if(status == true)
		printf("imu_set_standby_z_accel_enabled: success\r\n");
	else
		printf("imu_set_standby_z_accel_enabled: failed\r\n");
}

void imu_set_standby_xyz_accel_enabled(bool enabled)
{
	imu_set_standby_x_accel_enabled(enabled);
	imu_set_standby_y_accel_enabled(enabled);
	imu_set_standby_z_accel_enabled(enabled);

}

void imu_set_dhpf_mode(uint8_t bandwidth)
{
	volatile bool status = false;

	status = writeBits(imu_address, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_ACCEL_HPF_BIT, MPU6050_ACONFIG_ACCEL_HPF_LENGTH, bandwidth);
}

void imu_set_power_on_delay(uint8_t delay)
{
	volatile bool status = false;

	status = writeBits(imu_address, MPU6050_RA_MOT_DETECT_CTRL, MPU6050_DETECT_ACCEL_ON_DELAY_BIT, MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH, delay);
}

void imu_set_int_zero_motion_enabled(bool enabled)
{
	bool status = writeBit(imu_address, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT, enabled);

	if(status == true)
		printf("imu_set_int_zero_motion_enabled: success\r\n");
	else
		printf("imu_set_int_zero_motion_enabled: failed\r\n");
}

void imu_wom_set_pwr_mgmt_1(void)
{
	volatile bool status = false;

	status = writeBit(imu_address, MPU9150_RA_PWR_MGMT_1, MPU9150_PWR1_CYCLE_BIT, 0);
	cph_millis_delay(10);

	status = writeBit(imu_address, MPU9150_RA_PWR_MGMT_1, MPU9150_PWR1_SLEEP_BIT, 0);
	cph_millis_delay(10);

	status = writeBit(imu_address, MPU9150_RA_PWR_MGMT_1, MPU9150_PWR1_STANDBY_BIT, 0);
}

void imu_wom_set_pwr_mgmt_2(void)
{
	volatile bool status = false;

	status = writeBit(imu_address, MPU9150_RA_PWR_MGMT_2, MPU9150_PWR2_STBY_XA_BIT, 0);
	cph_millis_delay(10);

	status = writeBit(imu_address, MPU9150_RA_PWR_MGMT_2, MPU9150_PWR2_STBY_YA_BIT, 0);
	cph_millis_delay(10);

	status = writeBit(imu_address, MPU9150_RA_PWR_MGMT_2, MPU9150_PWR2_STBY_ZA_BIT, 0);
	cph_millis_delay(10);

	status = writeBit(imu_address, MPU9150_RA_PWR_MGMT_2, MPU9150_PWR2_STBY_XG_BIT, 1);
	cph_millis_delay(10);

	status = writeBit(imu_address, MPU9150_RA_PWR_MGMT_2, MPU9150_PWR2_STBY_YG_BIT, 1);
	cph_millis_delay(10);

	status = writeBit(imu_address, MPU9150_RA_PWR_MGMT_2, MPU9150_PWR2_STBY_ZG_BIT, 1);
}



//void imu_wom_set_accel_lpf(void)
//{
//	// pg. 7, 15 mpu-9250-register-map.pdf
//
//	// Set Accel LPF setting to 184 Hz Bandwidth
//	volatile bool status = false;
//
//	uint8_t accel_fchoice_b = (1 << 3);
//	uint8_t a_dlpfcfg = (1 << 0);
//
//	status = writeBits(imu_address, MPU9150_RA_FF_THR, 3, 4, (accel_fchoice_b | a_dlpfcfg) );
//
//}

void imu_wom_set_accel_lpf(void)
{

	volatile bool status = false;

	status = writeBit(imu_address, MPU9150_RA_FF_THR, MPU9150_DLPF_FCHOICE_B_BIT, 1);
	cph_millis_delay(10);

	status = writeBit(imu_address, MPU9150_RA_FF_THR, MPU9150_DLPF_A_DLPFCFG_BIT, 1);
	cph_millis_delay(10);


}

void imu_wom_enable_motion_interrupt(void)
{
	volatile bool status = false;

	status = writeBit(imu_address, MPU9150_RA_INT_ENABLE, MPU9150_WOM_EN_BIT, 1);
}


void imu_wom_enable_accel_hardware_intel(void)
{
	volatile bool status = false;

	status = writeBit(imu_address, MPU9150_RA_MOT_DETECT_CTRL, MPU9150_ACCEL_INTEL_EN, 1);
	cph_millis_delay(10);

	status = writeBit(imu_address, MPU9150_RA_MOT_DETECT_CTRL, MPU9150_ACCEL_INTEL_MODE, 1);
}

void imu_wom_set_wakeup_frequency(uint8_t frequency)
{
	volatile bool status = false;

	status = writeBits(imu_address, MPU6050_RA_PWR_MGMT_2, MPU6050_PWR2_LP_WAKE_CTRL_BIT, MPU6050_PWR2_LP_WAKE_CTRL_LENGTH, frequency);
}

void imu_wom_enable_cycle_mode()
{
	volatile bool status = false;

	status = writeBit(imu_address, MPU9150_RA_PWR_MGMT_1, MPU9150_PWR1_CYCLE_BIT, 1);
}



bool get_wake_cycle_enabled(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBit(imu_address, MPU9150_RA_PWR_MGMT_1, MPU9150_PWR1_CYCLE_BIT, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];
}

bool get_sleep_enabled(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBit(imu_address, MPU9150_RA_PWR_MGMT_1, MPU9150_PWR1_SLEEP_BIT, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];
}

bool get_standby_enabled(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBit(imu_address, MPU9150_RA_PWR_MGMT_1, MPU9150_PWR1_STANDBY_BIT, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];
}

bool get_temp_sensor_enabled(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBit(imu_address, MPU9150_RA_PWR_MGMT_1, MPU9150_PWR1_TEMP_DIS_BIT, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];
}


uint8_t get_clock_source(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBits(imu_address, MPU9150_RA_PWR_MGMT_1, MPU9150_PWR1_CLKSEL_BIT, MPU9150_PWR1_CLKSEL_LENGTH, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];

}

uint8_t get_wake_frequency(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBits(imu_address, MPU9150_RA_PWR_MGMT_2, MPU9150_PWR2_LP_WAKE_CTRL_BIT, MPU9150_PWR2_LP_WAKE_CTRL_LENGTH, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];
}

bool get_standby_x_accel_enabled(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBit(imu_address, MPU9150_RA_PWR_MGMT_2, MPU9150_PWR2_STBY_XA_BIT, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];
}
bool get_standby_y_accel_enabled(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBit(imu_address, MPU9150_RA_PWR_MGMT_2, MPU9150_PWR2_STBY_YA_BIT, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];
}

bool get_standby_z_accel_enabled(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBit(imu_address, MPU9150_RA_PWR_MGMT_2, MPU9150_PWR2_STBY_ZA_BIT, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];
}

bool get_standby_x_gyro_enabled(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBit(imu_address, MPU9150_RA_PWR_MGMT_2, MPU9150_PWR2_STBY_XG_BIT, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];
}

bool get_standby_y_gyro_enabled(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBit(imu_address, MPU9150_RA_PWR_MGMT_2, MPU9150_PWR2_STBY_YG_BIT, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];
}

bool get_standby_z_gyro_enabled(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBit(imu_address, MPU9150_RA_PWR_MGMT_2, MPU9150_PWR2_STBY_ZG_BIT, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];
}

uint8_t get_motion_interrupt_enabled(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readByte(imu_address, MPU9150_RA_INT_ENABLE, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];
}


bool get_accel_intel_enabled(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBit(imu_address, MPU9150_RA_MOT_DETECT_CTRL, MPU9150_ACCEL_INTEL_EN, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];
}

bool get_accel_intel_mode(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

	readBit(imu_address, MPU9150_RA_MOT_DETECT_CTRL, MPU9150_ACCEL_INTEL_MODE, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];
}

uint8_t get_motion_threshold(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));


	readByte(imu_address, MPU9150_RA_MOT_THR, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
	return imu_buffer[0];
}

