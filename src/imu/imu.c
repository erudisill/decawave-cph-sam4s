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

volatile bool wake_event_received = false;

uint8_t imu_address = IMU_ADDRESS;
static uint8_t imu_buffer[I2C_BUFFER_LENGTH] = {0};

static volatile uint8_t irq_set = 0x00;
static volatile uint32_t int_count = 0;

static void irq_handler(uint32_t id, uint32_t mask) {

	wake_event_received = true;
//	int_count++;
//	do {
//		dwt_isr();
//	} while (cph_deca_isr_is_detected() == 1);
}

static void irq_init(void) {

	pio_configure_pin(IMU_IRQ_IDX, IMU_IRQ_FLAGS);
	pio_pull_down(IMU_IRQ_PIO, IMU_IRQ_MASK, true);
	pio_handler_set(IMU_IRQ_PIO, IMU_IRQ_PIO_ID, IMU_IRQ_MASK, IMU_IRQ_ATTR, irq_handler);
	pio_enable_interrupt(IMU_IRQ_PIO, IMU_IRQ_MASK);

	pio_handler_set_priority(IMU_IRQ_PIO, IMU_IRQ_IRQ, 0);
	pmc_enable_periph_clk(IMU_IRQ_PIO_ID);
}


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



void imu_init_wom(void)
{
	imu_init();
	cph_millis_delay(10);

	imu_wom_set_pwr_mgmt_1();
	cph_millis_delay(10);

	imu_wom_set_pwr_mgmt_2();
	cph_millis_delay(10);

	imu_wom_set_accel_lpf();
	cph_millis_delay(10);

	imu_wom_enable_motion_interrupt();
	cph_millis_delay(10);

	imu_wom_enable_accel_hardware_intel();
	cph_millis_delay(10);

	imu_set_motion_detection_threshold(2);
	cph_millis_delay(10);

	imu_wom_set_wakeup_frequency(3);
	cph_millis_delay(10);

	imu_wom_enable_cycle_mode();

}

void imu_init_lowpower_motion_detection2(void)
{
	imu_init();

	imu_set_power_on_delay(3);
	cph_millis_delay(10);

	imu_set_int_zero_motion_enabled(false);
	cph_millis_delay(10);

	imu_set_dhpf_mode(MPU6050_DHPF_5);
	cph_millis_delay(10);

	imu_set_motion_detection_threshold(2);
	cph_millis_delay(10);

	imu_set_zero_motion_detection_threshold(2); // 2, 156
	cph_millis_delay(10);

	imu_set_motion_detection_duration(40); // 40, 80
	cph_millis_delay(10);

	imu_set_zero_motion_detection_duration(1); // 0, 2
	cph_millis_delay(10);

}

void imu_init(void)
{
	pmc_enable_periph_clk(IMU_TWI_ID);
	i2c_init(IMU_TWI);
	i2c_begin();

	irq_init();

	memset(imu_buffer, 0, sizeof(imu_buffer));

}

//void imu_init(void)
//{
//	pmc_enable_periph_clk(IMU_TWI_ID);
//	i2c_init(IMU_TWI);
//	i2c_begin();
//
//	memset(imu_buffer, 0, sizeof(imu_buffer));
//
////	imu_reset();
////	cph_millis_delay(10);
//
//	imu_set_clock_source(MPU6050_CLOCK_PLL_XGYRO);
//	cph_millis_delay(10);
//
//	imu_set_full_scale_gyro_range(MPU6050_GYRO_FS_250);
//	cph_millis_delay(10);
//
//	imu_set_full_scale_accel_range(MPU6050_ACCEL_FS_2);
//	cph_millis_delay(10);
//
//	imu_set_sleep_enabled(false);
//	cph_millis_delay(10);
//
//	imu_set_int_enabled(IMU_INTERRUPT_ENABLE);
//	cph_millis_delay(10);
//
//}

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

	readBits(imu_address, MPU9150_RA_CONFIG, MPU9150_CFG_DLPF_CFG_BIT, MPU9150_CFG_DLPF_CFG_LENGTH, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];
}

uint8_t imu_get_accel_config2(void)
{
	memset(imu_buffer, 0, sizeof(imu_buffer));

//	readBit(imu_address, MPU9150_RA_FF_THR
//	readBits(imu_address, MPU9150_RA_FF_THR, 0, 5, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);
//	readByte(imu_address, MPU9150_RA_FF_THR, imu_buffer, I2CDEV_DEFAULT_READ_TIMEOUT);

	return imu_buffer[0];
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

	status = writeBits(imu_address, MPU9150_RA_FF_THR, MPU9150_DLPF_A_DLPFCFG_BIT, MPU9150_DLPF_A_DLPFCFG_LENGTH, 1);


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


void run_imu_test(void)
{
	// *** begin test imu ***
	// *
	// *
	pio_set_pin_high(LED_STATUS0_IDX);
	while(true) {

		uint8_t c = 0x00;

		if (cph_usb_data_ready()) {
			cph_usb_data_read(&c);
		}

		if (c == '?') {
			TRACE("wake on motion settings:\r\n");

			TRACE("imu_get_accel_config2: %d\r\n", imu_get_accel_config2());
			TRACE("imu_get_wake_cycle_enabled: %d\r\n", get_wake_cycle_enabled());
			TRACE("imu_get_sleep_enabled: %d\r\n", get_sleep_enabled());
			TRACE("imu_get_standby_enabled: %d\r\n", get_standby_enabled());
			TRACE("imu_get_standby_x_accel_enabled: %d\r\n", get_standby_x_accel_enabled());
			TRACE("imu_get_standby_y_accel_enabled: %d\r\n", get_standby_y_accel_enabled());
			TRACE("imu_get_standby_z_accel_enabled: %d\r\n", get_standby_z_accel_enabled());
			TRACE("imu_get_standby_x_gyro_enabled: %d\r\n", get_standby_x_gyro_enabled());
			TRACE("imu_get_standby_y_gyro_enabled: %d\r\n", get_standby_y_gyro_enabled());
			TRACE("imu_get_standby_z_gyro_enabled: %d\r\n", get_standby_z_gyro_enabled());
			TRACE("imu_get_wake_frequency: %02x\r\n", get_wake_frequency());
			TRACE("imu_get_dlpf_mode: %02x\r\n", imu_get_dlpf_mode());
			TRACE("imu_get_motion_interrupt_enabled: %02x\r\n", get_motion_interrupt_enabled());
//			TRACE("imu_get_temp_sensor_enabled: %d\r\n", get_temp_sensor_enabled());


			TRACE("\r\n");
		}

		pio_toggle_pin(LED_STATUS0_IDX);

		if (wake_event_received) {
			wake_event_received = false;
			TRACE("WAKE EVENT RECEIVED\r\n");
		}
//		gimbal_tick();
		cph_millis_delay(125);
	}
	pio_set_pin_high(LED_STATUS0_IDX);
	// *
	// *
	// *** end test imu ***
}


// Configure the motion detection control for low power accelerometer mode
//void LowPowerAccelOnlyMPU6050()
//{
//
//// The sensor has a high-pass filter necessary to invoke to allow the sensor motion detection algorithms work properly
//// Motion detection occurs on free-fall (acceleration below a threshold for some time for all axes), motion (acceleration
//// above a threshold for some time on at least one axis), and zero-motion toggle (acceleration on each axis less than a
//// threshold for some time sets this flag, motion above the threshold turns it off). The high-pass filter takes gravity out
//// consideration for these threshold evaluations; otherwise, the flags would be set all the time!
//
//  uint8_t c = readByte(MPU6050_ADDRESS, PWR_MGMT_1);
//  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c & ~0x30); // Clear sleep and cycle bits [5:6]
//  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c |  0x30); // Set sleep and cycle bits [5:6] to zero to make sure accelerometer is running
//
//  c = readByte(MPU6050_ADDRESS, PWR_MGMT_2);
//  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c & ~0x38); // Clear standby XA, YA, and ZA bits [3:5]
//  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c |  0x00); // Set XA, YA, and ZA bits [3:5] to zero to make sure accelerometer is running
//
//  c = readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
//  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
//// Set high-pass filter to 0) reset (disable), 1) 5 Hz, 2) 2.5 Hz, 3) 1.25 Hz, 4) 0.63 Hz, or 7) Hold
//  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG,  c | 0x00);  // Set ACCEL_HPF to 0; reset mode disbaling high-pass filter
//
//  c = readByte(MPU6050_ADDRESS, CONFIG);
//  writeByte(MPU6050_ADDRESS, CONFIG, c & ~0x07); // Clear low-pass filter bits [2:0]
//  writeByte(MPU6050_ADDRESS, CONFIG, c |  0x00);  // Set DLPD_CFG to 0; 260 Hz bandwidth, 1 kHz rate
//
//  c = readByte(MPU6050_ADDRESS, INT_ENABLE);
//  writeByte(MPU6050_ADDRESS, INT_ENABLE, c & ~0xFF);  // Clear all interrupts
//  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x40);  // Enable motion threshold (bits 5) interrupt only
//
//// Motion detection interrupt requires the absolute value of any axis to lie above the detection threshold
//// for at least the counter duration
//  writeByte(MPU6050_ADDRESS, MOT_THR, 0x80); // Set motion detection to 0.256 g; LSB = 2 mg
//  writeByte(MPU6050_ADDRESS, MOT_DUR, 0x01); // Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate
//
//  delay (100);  // Add delay for accumulation of samples
//
//  c = readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
//  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
//  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c |  0x07);  // Set ACCEL_HPF to 7; hold the initial accleration value as a referance
//
//  c = readByte(MPU6050_ADDRESS, PWR_MGMT_2);
//  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c & ~0xC7); // Clear standby XA, YA, and ZA bits [3:5] and LP_WAKE_CTRL bits [6:7]
//  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c |  0x47); // Set wakeup frequency to 5 Hz, and disable XG, YG, and ZG gyros (bits [0:2])
//
//  c = readByte(MPU6050_ADDRESS, PWR_MGMT_1);
//  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c & ~0x20); // Clear sleep and cycle bit 5
//  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c |  0x20); // Set cycle bit 5 to begin low power accelerometer motion interrupts
//
//}
