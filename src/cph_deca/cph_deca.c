/*
 * cph_deca.c
 *
 *  Created on: Jan 11, 2016
 *      Author: ericrudisill
 */

#include <string.h>
#include <cph_deca.h>
#include <deca_regs.h>
#include <cph_queue.h>

static uint8 frame_seq_nb = 0;
static cph_queue_info_t event_queue;
static cph_deca_event_t events[CPH_MAX_EVENTS];

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
	status_reg = cph_deca_wait_for_tx_finished();
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	frame_seq_nb++;

	return status_reg;
}

uint32_t cph_deca_send_delayed() {
	uint32_t status_reg;

	int result = dwt_starttx(DWT_START_TX_DELAYED);
	if (result == DWT_SUCCESS) {
		status_reg = cph_deca_wait_for_tx_finished();
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
		status_reg = cph_deca_wait_for_tx_finished();
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
		// Frame sent, now wait for response
		status_reg = cph_deca_wait_for_rx_finished();
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	} else {
		TRACE("ERROR: dwt_starttx response returned %d - too late!\r\n", result);
	}
	frame_seq_nb++;

	return status_reg;
}

uint32_t cph_deca_send_response_expected() {
	uint32_t status_reg;

	dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
	status_reg = cph_deca_wait_for_rx_finished();
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
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

void cph_deca_init_device() {
	dwt_txconfig_t txconfig;

	// Setup DECAWAVE
	reset_DW1000();
	spi_set_rate_low();
	dwt_initialise(DWT_LOADUCODE);
	spi_set_rate_high();

	dwt_configure(&cph_config->dwt_config);

//	txconfig.PGdly = 0xC2;			// for channel 2
//	txconfig.power = 0x07274767;	// smart power, channel 2, 64MHz
//	dwt_setsmarttxpower(1);
//	dwt_configuretxrf(&txconfig);

	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);

	// Clear CLKPLL_LL
	dwt_write32bitreg(SYS_STATUS_ID, 0x02000000);
}

void cph_deca_init_network(uint16_t panid, uint16_t shortid) {
	// Configure network parameters
	dwt_setpanid(panid);
	dwt_setaddress16(shortid);
	dwt_enableframefilter(DWT_FF_DATA_EN);
}


void cph_deca_isr_handler(uint32_t id, uint32_t mask) {
	do {
		dwt_isr();
	} while (cph_deca_isr_is_detected() == 1);
}

void cph_deca_isr_init(void) {

	pio_configure_pin(DW_IRQ_IDX, DW_IRQ_FLAGS);
	pio_pull_down(DW_IRQ_PIO, DW_IRQ_MASK, true);
	pio_handler_set(DW_IRQ_PIO, DW_IRQ_PIO_ID, DW_IRQ_MASK, DW_IRQ_ATTR, cph_deca_isr_handler);
	pio_enable_interrupt(DW_IRQ_PIO, DW_IRQ_MASK);

	pio_handler_set_priority(DW_IRQ_PIO, DW_IRQ_IRQ, 0);

	pmc_enable_periph_clk(DW_IRQ_PIO_ID);
}

void cph_deca_isr_configure(void) {
	cph_queue_init(&event_queue, sizeof(cph_deca_event_t), CPH_MAX_EVENTS, events);

	dwt_setcallbacks(0, cph_deca_rxcallback);
//	dwt_setinterrupt(
//			DWT_INT_TFRS | DWT_INT_RFCG
//					| ( DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/),
//			1);
	dwt_setinterrupt(
			DWT_INT_TFRS | DWT_INT_RFCG
					| ( DWT_INT_RFTO ),
			1);
}

void cph_deca_rxcallback(const dwt_callback_data_t *rxd) {
	cph_deca_event_t ev;

	memcpy(&ev.info, rxd, sizeof(dwt_callback_data_t));

	if (rxd->event == DWT_SIG_RX_OKAY) {
//		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
		if (rxd->datalength <= CPH_MAX_MSG_SIZE) {
			ev.status = CPH_EVENT_RCV;
			dwt_readrxdata(ev.data, rxd->datalength, 0);
			//TODO: Capture timestamp here and stuff in event struct
			cph_queue_push(&event_queue, &ev);
			dwt_rxenable(0);
		}
	}
	else {
		ev.status = CPH_EVENT_ERR;
		cph_queue_push(&event_queue, &ev);
		// Auto enable should be on, so don't bother dwt_rxenable
	}
}

bool cph_deca_get_event(cph_deca_event_t * event) {
	if (cph_queue_pop(&event_queue, event) == CPH_QUEUE_OK) {
		return true;
	} else {
		return false;
	}
}
