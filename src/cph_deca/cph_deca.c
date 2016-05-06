/*
 * cph_deca.c
 *
 *  Created on: Jan 11, 2016
 *      Author: ericrudisill
 */

#include <string.h>
#include <cph.h>
#include <cph_deca.h>
#include <deca_regs.h>
#include <cph_queue.h>

static uint8 frame_seq_nb = 0;
static cph_queue_info_t event_queue;
static cph_deca_event_t events[CPH_MAX_EVENTS];

volatile uint8_t wait_event = 0xff;

static uint64_t get_rx_timestamp_u64(void) {
	uint8 ts_tab[5];
	uint64_t ts = 0;
	int i;
	dwt_readrxtimestamp(ts_tab);
	for (i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

static uint64_t get_tx_timestamp_u64(void) {
	uint8 ts_tab[5];
	uint64_t ts = 0;
	int i;
	dwt_readtxtimestamp(ts_tab);
	for (i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

uint32_t cph_deca_wait_for_tx_finished(int timeout_ms) {
	uint8_t dummy_signal = 0x00;
	return cph_deca_wait_for_tx_finished_signal(timeout_ms, &dummy_signal);
}

uint32_t cph_deca_wait_for_tx_finished_signal(int timeout_ms, volatile uint8_t * signal) {
	uint32_t status_reg;
	uint32_t start_ms = cph_get_millis();
	uint32_t elapsed = 0;
	while (cph_deca_isr_is_detected() == false) {
		if (*signal == 0xFF)
			return 0;
		elapsed = cph_get_millis() - start_ms;
		if (elapsed > timeout_ms)
			return 0;
	}
	status_reg = dwt_read32bitreg(SYS_STATUS_ID);
	return status_reg;
}

uint32_t cph_deca_wait_for_rx_finished(int timeout_ms) {
	uint8_t dummy_signal = 0x00;
	return cph_deca_wait_for_rx_finished_signal(timeout_ms, &dummy_signal);
}

uint32_t cph_deca_wait_for_rx_finished_signal(int timeout_ms, volatile uint8_t * signal) {
	uint32_t status_reg;
	uint32_t start_ms = cph_get_millis();
	uint32_t elapsed = 0;
	while (cph_deca_isr_is_detected() == false) {
		if (*signal == 0xFF)
			return 0;
		elapsed = cph_get_millis() - start_ms;
		if (elapsed > timeout_ms)
			return 0;
	}
	status_reg = dwt_read32bitreg(SYS_STATUS_ID);
	return status_reg;
}

void cph_deca_load_frame(cph_deca_msg_header_t * hdr, uint16_t size) {
	// Write message to frame buffer
	hdr->seq = frame_seq_nb;
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	dwt_writetxdata(size, (uint8_t*) hdr, 0);
	dwt_writetxfctrl(size, 0);
}

uint32_t cph_deca_send_immediate() {
	uint32_t status_reg;

	dwt_starttx(DWT_START_TX_IMMEDIATE);
	status_reg = cph_deca_wait_for_tx_finished(DEFAULT_TX_TIMEOUT);
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	frame_seq_nb++;

	return status_reg;
}

uint32_t cph_deca_send_delayed() {
	uint32_t status_reg;

	int result = dwt_starttx(DWT_START_TX_DELAYED);
	if (result == DWT_SUCCESS) {
		status_reg = cph_deca_wait_for_tx_finished(DEFAULT_TX_TIMEOUT);
	} else {
		TRACE("ERROR: dwt_starttx response returned %d - too late!\r\n", result);
	}
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	frame_seq_nb++;

	return status_reg;
}

uint32_t cph_deca_send_delayed_response_expected() {
	uint32_t status_reg;

	int result = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
	if (result == DWT_SUCCESS) {
		status_reg = cph_deca_wait_for_tx_finished(DEFAULT_TX_TIMEOUT);
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
		// Frame sent, now wait for response
		status_reg = cph_deca_wait_for_rx_finished(DEFAULT_RX_TIMEOUT);
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	} else {
		TRACE("ERROR: dwt_starttx response returned %d - too late!\r\n", result);
	}
	frame_seq_nb++;

	return status_reg;
}

//uint32_t cph_deca_send_response_expected() {
//	uint32_t status_reg;
//
//	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
//	status_reg = cph_deca_wait_for_rx_finished();
//	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
//	frame_seq_nb++;
//
//	return status_reg;
//}
uint32_t cph_deca_send_response_expected() {
	uint32_t status_reg;

	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
	status_reg = cph_deca_wait_for_tx_finished(DEFAULT_TX_TIMEOUT);
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	status_reg = cph_deca_wait_for_rx_finished(DEFAULT_RX_TIMEOUT);
	frame_seq_nb++;

	return status_reg;
}

cph_deca_msg_header_t * cph_deca_read_frame(uint8_t * rx_buffer, uint32_t *frame_len) {

	/* Clear good RX frame event in the DW1000 status register. */
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

	/* A frame has been received, read it into the local buffer. */
	*frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
	if (*frame_len <= CPH_MAX_MSG_SIZE) {
		dwt_readrxdata(rx_buffer, *frame_len, 0);
		return (cph_deca_msg_header_t*)rx_buffer;
	}
	else {
		return 0;
	}

}

void cph_deca_force_wakeup() {
	reset_DW1000();
	spi_set_rate_low();
	uint32_t id = dwt_readdevid();
	if (id == 0xFFFFFFFF) {
		TRACE("DW asleep..waking\r\n");
		// asleep, wakeup
		pio_set_pin_high(DW_WAKEUP_PIO_IDX);
		cph_millis_delay(1);
		pio_set_pin_low(DW_WAKEUP_PIO_IDX);
		cph_millis_delay(1);
	}
}

void cph_deca_init_device() {
	dwt_txconfig_t txconfig;

	TRACE("ant dly: %d %d\r\n", RX_ANT_DLY, TX_ANT_DLY);

	// Setup DECAWAVE
	reset_DW1000();
	spi_set_rate_low();

	dwt_initialise(DWT_LOADUCODE);
	spi_set_rate_high();

	dwt_configure(&cph_config->dwt_config);

	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);

	TRACE("ant dly: %d %d\r\n", RX_ANT_DLY, TX_ANT_DLY);

	// Clear CLKPLL_LL
	dwt_write32bitreg(SYS_STATUS_ID, 0x02000000);
}

void cph_deca_init_network(uint16_t panid, uint16_t shortid) {
	// Configure network parameters
	dwt_setpanid(panid);
	dwt_setaddress16(shortid);
	dwt_enableframefilter(DWT_FF_DATA_EN);
}


//void cph_deca_isr_handler(uint32_t id, uint32_t mask) {
//	do {
//		dwt_isr();
//	} while (cph_deca_isr_is_detected() == 1);
//}

void cph_deca_isr_handler(uint32_t id, uint32_t mask) {
	wait_event = 0x00;
}

void cph_deca_isr_init(void) {

	pio_configure_pin(DW_IRQ_IDX, DW_IRQ_FLAGS);
	pio_pull_down(DW_IRQ_PIO, DW_IRQ_MASK, true);
	pio_handler_set(DW_IRQ_PIO, DW_IRQ_PIO_ID, DW_IRQ_MASK, DW_IRQ_ATTR, cph_deca_isr_handler);
	pio_enable_interrupt(DW_IRQ_PIO, DW_IRQ_MASK);

	pio_handler_set_priority(DW_IRQ_PIO, DW_IRQ_IRQ, 0);

	pmc_enable_periph_clk(DW_IRQ_PIO_ID);
}

void cph_deca_isr_configure() {
	cph_queue_init(&event_queue, sizeof(cph_deca_event_t), CPH_MAX_EVENTS, events);

//	dwt_setcallbacks(cph_deca_txcallback, cph_deca_rxcallback);

//	dwt_setinterrupt(
//			DWT_INT_TFRS | DWT_INT_RFCG
//					| ( DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/),
//			1);
	dwt_setinterrupt(
			DWT_INT_TFRS | DWT_INT_RFCG
					| ( DWT_INT_RFTO ),
			1);
}



void cph_deca_txcallback(const dwt_callback_data_t * txd) {
	cph_deca_event_t ev;

	memcpy(&ev.info, txd, sizeof(dwt_callback_data_t));

	if (txd->event == DWT_SIG_TX_DONE) {
		ev.status = CPH_EVENT_TX;
		ev.timestamp = get_tx_timestamp_u64();
		cph_queue_push(&event_queue, &ev);
	}
	else {
		ev.status = CPH_EVENT_ERR;
		cph_queue_push(&event_queue, &ev);
		// Auto enable should be on, so don't bother dwt_rxenable
	}

	TRACE("%02X\r\n", ev.status);
}

void cph_deca_rxcallback(const dwt_callback_data_t *rxd) {
	cph_deca_event_t ev;

	memcpy(&ev.info, rxd, sizeof(dwt_callback_data_t));

	if (rxd->event == DWT_SIG_RX_OKAY) {
		if (rxd->datalength <= CPH_MAX_MSG_SIZE) {
			ev.status = CPH_EVENT_RX;
			dwt_readrxdata(ev.data, rxd->datalength, 0);
			ev.timestamp = get_rx_timestamp_u64();
			cph_queue_push(&event_queue, &ev);
		}
	}
	else {
		ev.status = CPH_EVENT_ERR;
		cph_queue_push(&event_queue, &ev);
		// Auto enable should be on, so don't bother dwt_rxenable
	}

	TRACE("%02X\r\n", ev.status);

}

bool cph_deca_get_event(cph_deca_event_t * event) {
	if (cph_queue_pop(&event_queue, event) == CPH_QUEUE_OK) {
		return true;
	} else {
		return false;
	}
}
