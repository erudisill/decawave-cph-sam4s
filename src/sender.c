/*
 * sender.c
 *
 *  Created on: Dec 9, 2015
 *      Author: ericrudisill
 */

#include <cph.h>

static cph_deca_msg_discover_announce_t tx_discover_msg = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		0xFFFF,			// mac.dest
		MAC_TAG_ID,		// mac.source
		FUNC_DISC_ANNO,	// functionCode
		0x0000			// mac_cs
		};



void RTT_Handler(void) {
	rtt_get_status(RTT);
}

static void sleep_all(uint32_t wait_ms) {
	dwt_entersleep();

	uint32_t rttv = rtt_read_timer_value(RTT);
	uint32_t wait_value = rttv + wait_ms;

	//TRACE("wait: %d %d\r\n", rttv, wait_value);
	cph_millis_delay(1);

	rtt_write_alarm_time(RTT, wait_value);
	pmc_sleep(SAM_PM_SMODE_WAIT);

	pio_set_pin_high(DW_WAKEUP_PIO_IDX);
	cph_millis_delay(1);
	pio_set_pin_low(DW_WAKEUP_PIO_IDX);
	cph_millis_delay(1);

	cph_deca_init_device();
	cph_deca_init_network(cph_config->panid, cph_config->shortid);
}

void sender_run(void) {

	uint32_t id = 0;
	uint32_t announce_coord_ts = 0;
	uint32_t elapsed = 0;
	uint32_t count = 0;


	reset_DW1000();
	spi_set_rate_low();
	id = dwt_readdevid();
	if (id == 0xFFFFFFFF) {
		TRACE("DW asleep..waking\r\n");
		// asleep, wakeup
		pio_set_pin_high(DW_WAKEUP_PIO_IDX);
		cph_millis_delay(1);
		pio_set_pin_low(DW_WAKEUP_PIO_IDX);
		cph_millis_delay(1);
	}

//	// Setup DW1000
//	dwt_txconfig_t txconfig;
//
//	// Setup DECAWAVE
//	reset_DW1000();
//	spi_set_rate_low();
//	dwt_initialise(DWT_LOADUCODE);
//	spi_set_rate_high();
//
//	dwt_configure(&cph_config->dwt_config);
//
//	dwt_setpanid();
//	dwt_setaddress16(0x1234);
//
//	// Clear CLKPLL_LL
//	dwt_write32bitreg(SYS_STATUS_ID, 0x02000000);

	cph_deca_init_device();
	cph_deca_init_network(cph_config->panid, cph_config->shortid);


	id = dwt_readdevid();
	printf("Device ID: %08X\r\n", id);

	uint8_t seq = 0;
	uint32_t status_reg;

	tx_discover_msg.header.panid = cph_config->panid;
	tx_discover_msg.header.source = cph_config->shortid;
//	tx_discover_msg.header.dest = cph_config->sender_target;

	dwt_configuresleep(DWT_PRESRV_SLEEP | DWT_CONFIG, DWT_WAKE_WK | DWT_SLP_EN);

//	rtt_init(RTT, 32768);
	rtt_init(RTT, 32);
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
	pmc_set_fast_startup_input(PMC_FSMR_RTTAL);


	while (1) {

		printf("sending %d\r\n", count++);

		// Write message to frame buffer
		tx_discover_msg.header.seq = seq++;
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
		dwt_writetxdata(sizeof(tx_discover_msg), (uint8_t*) &tx_discover_msg, 0);
		dwt_writetxfctrl(sizeof(tx_discover_msg), 0);

		dwt_starttx(DWT_START_TX_IMMEDIATE);

		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS)) {
		};
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

//		dwt_entersleep();
//
//		rtt_write_alarm_time(RTT, rtt_read_timer_value(RTT) + 200);
//		pmc_sleep(SAM_PM_SMODE_WAIT);

//		cph_millis_delay(cph_config->sender_period);

//		pio_set_pin_high(DW_WAKEUP_PIO_IDX);
//		cph_millis_delay(1);
//		pio_set_pin_low(DW_WAKEUP_PIO_IDX);
//		cph_millis_delay(10);

//		sleep_all(1000);

		sleep_all(250);
	}
}
