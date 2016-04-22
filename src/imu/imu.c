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
#include "imu_api.h"
#include "imu.h"

volatile bool wake_event_received = false;

static void irq_init(void);
static void irq_handler(uint32_t id, uint32_t mask);

void PMC_Handler(void)
{
	TRACE("PMC_Handler\r\n");
	if(pmc_get_status() & PMC_SR_CFDEV) {

	}
}

void RTT_handler(void)
{
	uint32_t ul_status;

	ul_status = rtt_get_status(RTT);

	if((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		TRACE("ALARM\r\n");
	}
}

static void irq_init(void) {

	pio_configure_pin(IMU_IRQ_IDX, IMU_IRQ_FLAGS);
	pio_pull_down(IMU_IRQ_PIO, IMU_IRQ_MASK, true);
	pio_handler_set(IMU_IRQ_PIO, IMU_IRQ_PIO_ID, IMU_IRQ_MASK, IMU_IRQ_ATTR, irq_handler);
	pio_enable_interrupt(IMU_IRQ_PIO, IMU_IRQ_MASK);

	pio_handler_set_priority(IMU_IRQ_PIO, IMU_IRQ_IRQ, 0);
	pmc_enable_periph_clk(IMU_IRQ_PIO_ID);
}

static void irq_handler(uint32_t id, uint32_t mask)
{
	wake_event_received = true;
}

void imu_init(void)
{
	pmc_enable_periph_clk(IMU_TWI_ID);
	i2c_init(IMU_TWI);
	i2c_begin();

	irq_init();

	imu_reset();
	cph_millis_delay(10);

//	imu_set_int_enabled(IMU_INTERRUPT_ENABLE);

	memset(imu_buffer, 0, sizeof(imu_buffer));
}

void imu_init_wom(void)
{
	imu_reset();
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

void imu_init_lowpower_motion_detection(void)
{
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

void imu_init_gimbal(void)
{
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


void imu_run_console(void)
{
	uint32_t ul_previous_time;

	rtt_sel_source(RTT, false);
	rtt_init(RTT, 32);
//	rtt_init(RTT, 32768);

	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));

	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);

	NVIC_EnableIRQ(PMC_IRQn);
	pmc_set_fast_startup_input(PMC_FSMR_RTTAL | PMC_FSMR_FSTT2);



	volatile uint32_t wait_ms = 5000;

	pio_set_pin_high(LED_STATUS0_IDX);
	while(true) {

		uint8_t c = 0x00;

		if (cph_stdio_dataready()) {
			cph_stdio_readc(&c);
		}

		if (c == '?') {
//			TRACE("wake on motion settings:\r\n");
//
//			TRACE("imu_get_wake_cycle_enabled: %d\r\n", get_wake_cycle_enabled());
//			TRACE("imu_get_sleep_enabled: %d\r\n", get_sleep_enabled());
//			TRACE("imu_get_standby_enabled: %d\r\n", get_standby_enabled());
//			TRACE("imu_get_standby_x_accel_enabled: %d\r\n", get_standby_x_accel_enabled());
//			TRACE("imu_get_standby_y_accel_enabled: %d\r\n", get_standby_y_accel_enabled());
//			TRACE("imu_get_standby_z_accel_enabled: %d\r\n", get_standby_z_accel_enabled());
//			TRACE("imu_get_standby_x_gyro_enabled: %d\r\n", get_standby_x_gyro_enabled());
//			TRACE("imu_get_standby_y_gyro_enabled: %d\r\n", get_standby_y_gyro_enabled());
//			TRACE("imu_get_standby_z_gyro_enabled: %d\r\n", get_standby_z_gyro_enabled());
//			imu_get_accel_config2();
//			TRACE("imu_get_motion_interrupt_enabled: 0x%02x\r\n", get_motion_interrupt_enabled());
//			TRACE("get_accel_intel_enabled: %d\r\n", get_accel_intel_enabled());
//			TRACE("get_accel_intel_mode: %d\r\n", get_accel_intel_mode());
//			TRACE("get_motion_threshold: %d\r\n", get_motion_threshold());
//			TRACE("imu_get_wake_frequency: 0x%02x\r\n", get_wake_frequency());


//			TRACE("0x1A CONFIG: %02x\r\n", imu_get_reg(MPU9150_RA_CONFIG));
//			TRACE("0x1B GYRO_CONFIG: %02x\r\n", imu_get_reg(MPU9150_RA_GYRO_CONFIG));
//			TRACE("0x1C ACCEL_CONFIG: %02x\r\n", imu_get_reg(MPU9150_RA_ACCEL_CONFIG));
//			TRACE("0x1D ACCEL_CONFIG 2: %02x\r\n", imu_get_reg(MPU9150_RA_FF_THR));
//			TRACE("0x1E LP_ACCEL_ODR: %02x\r\n", imu_get_reg(MPU9150_RA_FF_DUR));
//			TRACE("0x1F WOM THR: %02x\r\n", imu_get_reg(MPU9150_RA_MOT_THR));
//			TRACE("0x38 INT_ENABLE: %02x\r\n", imu_get_int_enabled());
//			cph_millis_delay(10);
//			TRACE("0x3A INT_STATUS: %02x\r\n", imu_get_reg(MPU9150_RA_INT_STATUS));
//			TRACE("0x69 MOT_DETECT_CTRL: %02x\r\n", imu_get_reg(MPU9150_RA_MOT_DETECT_CTRL));
//			TRACE("0x6B PWR_MGMT_1: %02x\r\n", imu_get_reg(MPU9150_RA_PWR_MGMT_1));
//			TRACE("0x6C PWR_MGMT_2: %02x\r\n", imu_get_reg(MPU9150_RA_PWR_MGMT_2));


			TRACE("0x6B PWR_MGMT_1 CYCLE_BIT: %02x\r\n", imu_get_reg_bit(MPU9150_RA_PWR_MGMT_1, MPU9150_PWR1_CYCLE_BIT));
			cph_millis_delay(10);
			TRACE("0x6B PWR_MGMT_1 SLEEP_BIT: %02x\r\n", imu_get_reg_bit(MPU9150_RA_PWR_MGMT_1, MPU9150_PWR1_SLEEP_BIT));
			cph_millis_delay(10);
			TRACE("0x6B PWR_MGMT_1 STANDBY_BIT: %02x\r\n", imu_get_reg_bit(MPU9150_RA_PWR_MGMT_1, MPU9150_PWR1_STANDBY_BIT));
			cph_millis_delay(10);
			TRACE("0x6B PWR_MGMT_1 CLKSEL[0]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_PWR_MGMT_1, 0));
			cph_millis_delay(10);
			TRACE("0x6B PWR_MGMT_1 CLKSEL[1]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_PWR_MGMT_1, 1));
			cph_millis_delay(10);
			TRACE("0x6B PWR_MGMT_1 CLKSEL[2]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_PWR_MGMT_1, 2));
			cph_millis_delay(10);
			TRACE("0x6C PWR_MGMT_2 bit[0]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_PWR_MGMT_2, 0));
			cph_millis_delay(10);
			TRACE("0x6C PWR_MGMT_2 bit[1]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_PWR_MGMT_2, 0));
			cph_millis_delay(10);
			TRACE("0x6C PWR_MGMT_2 bit[2]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_PWR_MGMT_2, 0));
			cph_millis_delay(10);
			TRACE("0x6C PWR_MGMT_2 bit[3]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_PWR_MGMT_2, 0));
			cph_millis_delay(10);
			TRACE("0x6C PWR_MGMT_2 bit[4]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_PWR_MGMT_2, 0));
			cph_millis_delay(10);
			TRACE("0x6C PWR_MGMT_2 bit[5]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_PWR_MGMT_2, 0));
			cph_millis_delay(10);

			TRACE("0x1D ACCEL_CONFIG 2 A_DLPF_CFG[0]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_FF_THR, MPU9150_DLPF_A_DLPFCFG_BIT));
			cph_millis_delay(10);
			TRACE("0x1D ACCEL_CONFIG 2 A_DLPF_CFG[1]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_FF_THR, (MPU9150_DLPF_A_DLPFCFG_BIT+1)));
			cph_millis_delay(10);
			TRACE("0x1D ACCEL_CONFIG 2 A_DLPF_CFG[2]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_FF_THR, (MPU9150_DLPF_A_DLPFCFG_BIT+2)));
			cph_millis_delay(10);
			TRACE("0x1D ACCEL_CONFIG 2 DLPF_FCHOICE_B_BIT: %02x\r\n", imu_get_reg_bit(MPU9150_RA_FF_THR, MPU9150_DLPF_FCHOICE_B_BIT));
			cph_millis_delay(10);
			TRACE("0x38 INT_ENABLE MPU9150_WOM_EN_BIT: %02x\r\n", imu_get_reg_bit(MPU9150_RA_INT_ENABLE, MPU9150_WOM_EN_BIT));
			cph_millis_delay(10);
			TRACE("0x69 INT_ENABLE MPU9150_ACCEL_INTEL_EN: %02x\r\n", imu_get_reg_bit(MPU9150_RA_MOT_DETECT_CTRL, MPU9150_ACCEL_INTEL_EN));
			cph_millis_delay(10);
			TRACE("0x69 INT_ENABLE MPU9150_ACCEL_INTEL_MODE: %02x\r\n", imu_get_reg_bit(MPU9150_RA_MOT_DETECT_CTRL, MPU9150_ACCEL_INTEL_MODE));
			cph_millis_delay(10);




			TRACE("\r\n");
		} else if (c == 's') {


//			static volatile uint32_t wait_ms = 5;

			TRACE("\r\nSLEEP Zzzzzzz.....\r\n");
			cph_millis_delay(500);

			volatile uint32_t rttv = rtt_read_timer_value(RTT);
			volatile uint32_t wait_value = rttv + wait_ms;

			cph_millis_delay(1);

			rtt_write_alarm_time(RTT, wait_value);

//			imu_wom_enable_cycle_mode();
			imu_set_sleep_enabled(true);

			cpu_irq_disable();
			pmc_sleep(SAM_PM_SMODE_WAIT);
			cpu_irq_enable();

			g_cph_millis += wait_ms;

			TRACE("DONE SLEEPING\r\n");

		} else if (c == 'c') {
			TRACE("configuring mpu9250...\r\n");
//			imu_init_lowpower_motion_detection();
			TRACE("configuration complete!\r\n");
			imu_init_wom();
//			imu_init_gimbal();
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
