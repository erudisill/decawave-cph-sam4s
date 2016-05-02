/*
 * sender.c
 *
 *  Created on: Dec 9, 2015
 *      Author: ericrudisill
 */

#include <cph.h>
#include <cph_deca.h>
#include <deca_sleep.h>

static cph_deca_msg_blink_t tx_blink_msg = {
	MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
	0,				// mac.seq
	MAC_PAN_ID,		// mac.panid
	0xFFFF,			// mac.dest
	MAC_TAG_ID,		// mac.source
	FUNC_CS_SYNC,	// functionCode
	0,				// blinkTxTs
	0x0000			// mac_cs
};

static void irq_handler(uint32 id, uint32 mask) {

}

static void irq_init(void) {
	pio_configure_pin(DW_IRQ_IDX, DW_IRQ_FLAGS);
	pio_pull_down(DW_IRQ_PIO, DW_IRQ_MASK, true);
	pio_handler_set(DW_IRQ_PIO, DW_IRQ_PIO_ID, DW_IRQ_MASK, DW_IRQ_ATTR, irq_handler);
	pio_enable_interrupt(DW_IRQ_PIO, DW_IRQ_MASK);

	pio_handler_set_priority(DW_IRQ_PIO, DW_IRQ_IRQ, 0);

	pmc_enable_periph_clk(DW_IRQ_PIO_ID);
}

uint64_t get_tx_timestamp(void) {
	uint8_t ts_tab[5];
	uint64_t ts = 0;
	int i;
	dwt_readtxtimestamp(ts_tab);
	for (i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

uint64_t get_sys_time(void) {
	uint8_t ts_tab[5];
	uint64_t ts = 0;
	int i;
	dwt_readsystime(ts_tab);
	for (i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

static void txcallback(const dwt_callback_data_t *rxd) {
}


void cs_sender_run(void) {

	uint32_t id = 0;
	uint32_t announce_coord_ts = 0;
	uint32_t elapsed = 0;
	uint32_t count = 0;
	uint32_t send_ts = 0;


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

	cph_deca_init_device();
	cph_deca_init_network(cph_config->panid, cph_config->shortid);


	id = dwt_readdevid();
	printf("Device ID: %08X\r\n", id);

	uint8_t seq = 0;
	uint32_t status_reg;

	tx_blink_msg.header.panid = cph_config->panid;
	tx_blink_msg.header.source = cph_config->shortid;

	dwt_configuresleep(DWT_PRESRV_SLEEP | DWT_CONFIG, DWT_WAKE_WK | DWT_SLP_EN);

	rtt_init(RTT, 32);
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
	pmc_set_fast_startup_input(PMC_FSMR_RTTAL);

	irq_init();
	dwt_setinterrupt( DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
	pio_enable_interrupt(DW_IRQ_PIO, DW_IRQ_MASK);

	dwt_setcallbacks(txcallback, 0);

	// Write message to frame buffer
	tx_blink_msg.header.seq = seq++;
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	dwt_writetxdata(sizeof(tx_blink_msg), (uint8_t*) &tx_blink_msg, 0);
	dwt_writetxfctrl(sizeof(tx_blink_msg), 0);

	uint32_t period = cph_config->sender_period;
	int result = 0;

	dwt_starttx(DWT_START_TX_IMMEDIATE);
	cph_deca_wait_for_tx_finished(DEFAULT_TX_TIMEOUT);
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
//	deca_sleep(period);

	uint64_t temp;

	while(1)
	{
		send_ts = ((get_tx_timestamp() + ((uint64_t)period * MS_TO_DWT_TIME)) >> 8);
		dwt_setdelayedtrxtime(send_ts);
//		dwt_starttx(DWT_START_TX_IMMEDIATE);
		result = dwt_starttx(DWT_START_TX_DELAYED);
		if(result != DWT_SUCCESS)
		{
			TRACE("RESULT: %d SEND_TS: %08X\r\n", result, send_ts);
			deca_sleep(3000);
		}
		else {
			cph_deca_wait_for_tx_finished(DEFAULT_TX_TIMEOUT);
			tx_blink_msg.header.seq = seq++;
			tx_blink_msg.blinkTxTs = send_ts;
			dwt_writetxdata(sizeof(tx_blink_msg), (uint8_t*) &tx_blink_msg, 0);
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
		}
	}
}
