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

/* Low-power accel wakeup rates. */
enum lp_accel_rate_e {
    INV_LPA_1_25HZ,
    INV_LPA_5HZ,
    INV_LPA_20HZ,
    INV_LPA_40HZ
};

/* Filter configurations. */
enum lpf_e {
    INV_FILTER_256HZ_NOLPF2 = 0,
    INV_FILTER_188HZ,
    INV_FILTER_98HZ,
    INV_FILTER_42HZ,
    INV_FILTER_20HZ,
    INV_FILTER_10HZ,
    INV_FILTER_5HZ,
    INV_FILTER_2100HZ_NOLPF,
    NUM_FILTER
};



volatile bool wake_event_received = false;
static uint32_t rtt_alarm_wait_ms;

static void irq_init(void);
static void irq_handler(uint32_t id, uint32_t mask);
static void imu_prepare_sleep(void);
void log_imu_settings(void);
void log_imu_registers(void);

static void imu_handler(uint32_t ul_id, uint32_t ul_mask);
void imu_enable_interrupts(void);
static volatile uint32_t g_ul_imu_interrupt = 1;

static void imu_handler(uint32_t ul_id, uint32_t ul_mask)
{
	if (IMU_IRQ_PIO_ID == ul_id && IMU_WAKEUP_MASK == ul_mask) {
		g_ul_imu_interrupt = 1;
	}
}

static void irq_handler(uint32_t id, uint32_t mask)
{
	wake_event_received = true;
}

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


static void irq_init(void)
{



}


void imu_init(void)
{
	pmc_enable_periph_clk(IMU_TWI_ID);

	i2c_init(IMU_TWI);
	i2c_begin();

	irq_init();

	imu_reset();
	cph_millis_delay(10);

	memset(imu_buffer, 0, sizeof(imu_buffer));
}

void imu_lp_accel_mode(uint8_t rate)
{
	imu_wom_set_int_latched(1);
	cph_millis_delay(10);
}

void imu_init_wom(void)
{
	imu_reset();
	cph_millis_delay(10);


	imu_set_int_enabled(0); // todo: new code
	cph_millis_delay(10);

	imu_wom_set_int_latched(1); // todo: new code
	cph_millis_delay(10);

	imu_wom_set_pwr_mgmt_1();
	cph_millis_delay(10);

	imu_wom_set_pwr_mgmt_2();
	cph_millis_delay(10);

	imu_wom_set_accel_lpf();
	cph_millis_delay(10);

	imu_wom_enable_motion_interrupt();
	cph_millis_delay(10);

//	imu_wom_enable_raw_dataready_interrupt();
//	cph_millis_delay(10);

	imu_wom_enable_accel_hardware_intel();
	cph_millis_delay(10);

	imu_set_motion_detection_threshold(2);
	cph_millis_delay(10);

	imu_wom_set_wakeup_frequency();
	cph_millis_delay(10);

	imu_wom_enable_cycle_mode();
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


static void imu_sleep(void)
{

	TRACE("\r\n\r\nGoing to sleep...\r\n");


	uint32_t ul_previous_time;

	pio_configure(PIOA, PIO_INPUT, PIO_PA2, 0);
	PIOA->PIO_PPDER |= PIO_PA2;

	rtt_sel_source(RTT, false);
	rtt_init(RTT, 32); // 32768

#define WAKEUP_BACKUP
//#define WAKEUP_WAIT


#ifdef WAKEUP_BACKUP
	supc_set_wakeup_mode(SUPC, SUPC_WUMR_RTTEN_ENABLE);
	supc_set_wakeup_inputs(SUPC, SUPC_WUIR_WKUPEN2_ENABLE, SUPC_WUIR_WKUPT2_HIGH);
#endif
#ifdef WAKEUP_WAIT
	pmc_set_fast_startup_input(PMC_FSMR_RTTAL | PMC_FSMR_FSTT0);
#endif

	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);


	volatile uint32_t rttv = rtt_read_timer_value(RTT);
	volatile uint32_t wait_value = rttv + IMU_RTT_ALARM_MS;

	cph_millis_delay(1);

	rtt_write_alarm_time(RTT, wait_value);

	imu_wom_enable_cycle_mode();

	cpu_irq_disable();



#ifdef WAKEUP_WAIT
	pmc_sleep(SAM_PM_SMODE_WAIT);
#endif
#ifdef WAKEUP_BACKUP
	pmc_sleep(SAM_PM_SMODE_BACKUP);
#endif

	cpu_irq_enable();

	cph_millis_delay(1);

	TRACE("Awake!\r\n");

}

void imu_run_console(void)
{

	pio_set_pin_high(LED_STATUS0_IDX);

	while(true) {

		uint8_t c = 0x00;

		if (cph_stdio_dataready()) {
			cph_stdio_readc(&c);
		}

		if (c == '?') {
			log_imu_settings();

		} else if (c == '*') {
			log_imu_registers();

		} else if (c == 's') {

			imu_wom_enable_cycle_mode();
			cph_millis_delay(100);
			imu_sleep();

		} else if (c == 'c') {
			TRACE("configuring mpu9250...\r\n");
			imu_init_wom();
			TRACE("configuration complete!\r\n");
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

}


void log_imu_settings(void)
{
	TRACE("0x6B PWR_MGMT_1 reg: %02x\r\n", imu_get_reg(MPU9150_RA_PWR_MGMT_1));
	cph_millis_delay(10);


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
	TRACE("0x69 MOT_DETECT_CTRL ACCEL_INTEL_EN: %02x\r\n", imu_get_reg_bit(MPU9150_RA_MOT_DETECT_CTRL, MPU9150_ACCEL_INTEL_EN));
	cph_millis_delay(10);
	TRACE("0x69 MOT_DETECT_CTRL ACCEL_INTEL_MODE: %02x\r\n", imu_get_reg_bit(MPU9150_RA_MOT_DETECT_CTRL, MPU9150_ACCEL_INTEL_MODE));
	cph_millis_delay(10);

	TRACE("0x1F WOM_THR bit[0]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_MOT_THR, 0));
	cph_millis_delay(10);
	TRACE("0x1F WOM_THR bit[1]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_MOT_THR, 1));
	cph_millis_delay(10);
	TRACE("0x1F WOM_THR bit[2]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_MOT_THR, 2));
	cph_millis_delay(10);
	TRACE("0x1F WOM_THR bit[3]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_MOT_THR, 3));
	cph_millis_delay(10);
	TRACE("0x1F WOM_THR bit[4]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_MOT_THR, 4));
	cph_millis_delay(10);
	TRACE("0x1F WOM_THR bit[5]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_MOT_THR, 5));
	cph_millis_delay(10);
	TRACE("0x1F WOM_THR bit[6]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_MOT_THR, 6));
	cph_millis_delay(10);
	TRACE("0x1F WOM_THR bit[7]: %02x\r\n", imu_get_reg_bit(MPU6050_RA_MOT_THR, 7));
	cph_millis_delay(10);

	TRACE("0x1E LP_ACCEL_ODR lposc_clkselbit[0]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_FF_DUR, 0));
	cph_millis_delay(10);
	TRACE("0x1E LP_ACCEL_ODR lposc_clkselbit[1]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_FF_DUR, 1));
	cph_millis_delay(10);
	TRACE("0x1E LP_ACCEL_ODR lposc_clkselbit[2]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_FF_DUR, 2));
	cph_millis_delay(10);
	TRACE("0x1E LP_ACCEL_ODR lposc_clkselbit[3]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_FF_DUR, 3));
	cph_millis_delay(10);

	TRACE("0x37 ACTL bit[0]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_INT_PIN_CFG, 0));
	cph_millis_delay(10);
	TRACE("0x37 ACTL bit[1]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_INT_PIN_CFG, 1));
	cph_millis_delay(10);
	TRACE("0x37 ACTL bit[2]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_INT_PIN_CFG, 2));
	cph_millis_delay(10);
	TRACE("0x37 ACTL bit[3]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_INT_PIN_CFG, 3));
	cph_millis_delay(10);
	TRACE("0x37 ACTL bit[4]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_INT_PIN_CFG, 4));
	cph_millis_delay(10);
	TRACE("0x37 ACTL bit[5]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_INT_PIN_CFG, 5));
	cph_millis_delay(10);
	TRACE("0x37 ACTL bit[6]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_INT_PIN_CFG, 6));
	cph_millis_delay(10);
	TRACE("0x37 ACTL bit[7]: %02x\r\n", imu_get_reg_bit(MPU9150_RA_INT_PIN_CFG, 7));
	cph_millis_delay(10);

	TRACE("0x38 INT_ENABLE WOM_EN_BIT: %02x\r\n", imu_get_reg_bit(MPU9150_RA_INT_ENABLE, MPU9150_WOM_EN_BIT));
	cph_millis_delay(10);

	TRACE("0x38 INT_ENABLE RAW_RDY_EN: %02x\r\n", imu_get_reg_bit(MPU9150_RA_INT_ENABLE, MPU9150_RAW_RDY_EN_BIT));
	cph_millis_delay(10);

	TRACE("\r\n");
}

void log_imu_registers(void)
{
	for (int i=0; i<=58; i++) {

		uint8_t reg = imu_get_reg(i);

		TRACE("addr: %02x bit[0]: %02x\r\n", i, (reg & (1 << 0)) ? 1 : 0);
		TRACE("addr: %02x bit[1]: %02x\r\n", i, (reg & (1 << 1)) ? 1 : 0);
		TRACE("addr: %02x bit[2]: %02x\r\n", i, (reg & (1 << 2)) ? 1 : 0);
		TRACE("addr: %02x bit[3]: %02x\r\n", i, (reg & (1 << 3)) ? 1 : 0);
		TRACE("addr: %02x bit[4]: %02x\r\n", i, (reg & (1 << 4)) ? 1 : 0);
		TRACE("addr: %02x bit[5]: %02x\r\n", i, (reg & (1 << 5)) ? 1 : 0);
		TRACE("addr: %02x bit[6]: %02x\r\n", i, (reg & (1 << 6)) ? 1 : 0);
		TRACE("addr: %02x bit[7]: %02x\r\n", i, (reg & (1 << 7)) ? 1 : 0);
		cph_millis_delay(10);
	}

	TRACE("\r\n\r\n");

	for (int i=105; i<=126; i++) {
		uint8_t reg = imu_get_reg(i);

		TRACE("addr: %02x bit[0]: %02x\r\n", i, (reg & (1 << 0)) ? 1 : 0);
		TRACE("addr: %02x bit[1]: %02x\r\n", i, (reg & (1 << 1)) ? 1 : 0);
		TRACE("addr: %02x bit[2]: %02x\r\n", i, (reg & (1 << 2)) ? 1 : 0);
		TRACE("addr: %02x bit[3]: %02x\r\n", i, (reg & (1 << 3)) ? 1 : 0);
		TRACE("addr: %02x bit[4]: %02x\r\n", i, (reg & (1 << 4)) ? 1 : 0);
		TRACE("addr: %02x bit[5]: %02x\r\n", i, (reg & (1 << 5)) ? 1 : 0);
		TRACE("addr: %02x bit[6]: %02x\r\n", i, (reg & (1 << 6)) ? 1 : 0);
		TRACE("addr: %02x bit[7]: %02x\r\n", i, (reg & (1 << 7)) ? 1 : 0);
		cph_millis_delay(10);
	}


}
