/*
 * twr_tag.c
 *
 *  Created on: Mar 15, 2016
 *      Author: ericrudisill
 */


#include <stdio.h>
#include <string.h>

#include <cph.h>
#include <cph_deca.h>
#include <cph_deca_range.h>
#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_sleep.h>

#define ANCHOR_ID		0x616A

/* Frames used in the ranging process.  */
static cph_deca_msg_discover_announce_t tx_discover_msg = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		0xFFFF,			// mac.dest
		MAC_TAG_ID,		// mac.source
		FUNC_DISC_ANNO,	// functionCode
		0x0000			// mac_cs
		};

static cph_deca_msg_pair_response_t tx_pair_msg = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		MAC_ANCHOR_ID,	// mac.dest
		MAC_TAG_ID,		// mac.source
		FUNC_PAIR_RESP,		// functionCode
		0x0000			// mac_cs
		};

static cph_deca_msg_range_report_t tx_range_results_msg = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		MAC_ANCHOR_ID,	// mac.dest
		MAC_TAG_ID,		// mac.source
		FUNC_RANGE_REPO,		// functionCode
		0,				// num ranges
		0,		// results
		0x0000			// mac_cs
		};


typedef unsigned long long uint64;

// Discovered anchors and their ranges
static cph_deca_anchor_range_t anchors[ANCHORS_MIN];
static unsigned int anchors_status;

/* Buffer to store received response message.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
static uint8 rx_buffer[CPH_MAX_MSG_SIZE];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;


#if 0
static void sleep_all(uint32_t wait_ms) {
	uint32_t ul_previous_time;

	dwt_entersleep();
	cph_millis_delay(5);

	supc_set_wakeup_mode(SUPC, SUPC_WUMR_RTTEN_ENABLE);

	/* Configure RTT for a 1 second tick interrupt */
	rtt_init(RTT, 32768);
	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));

	/* Enable RTT alarms interrupt to return from backup mode */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);

	rtt_write_alarm_time(RTT, 5);

	MATRIX->CCFG_SYSIO = 0xffffffff;
	pio_configure(PIOA, PIO_OUTPUT_1, 0xffffffff, PIO_OPENDRAIN);
	pio_configure(PIOB, PIO_OUTPUT_1, 0xffffffff, PIO_OPENDRAIN);
	pio_configure(PIOC, PIO_OUTPUT_1, 0xffffffff, PIO_OPENDRAIN);
	pmc_disable_all_periph_clk();
	pmc_switch_mck_to_sclk(PMC_MCKR_PRES_CLK_1);
	pmc_switch_mainck_to_fastrc(CKGR_MOR_MOSCRCF_4_MHz);
	pmc_osc_disable_xtal(0);
	pmc_disable_pllack();
	pmc_disable_pllbck();
	pmc_switch_mck_to_mainck(PMC_MCKR_PRES_CLK_1);


	pmc_enable_backupmode();

	// END OF THE LINE

}

#else

static void sleep_all(uint32_t wait_ms) {
	dwt_entersleep();

#if 1
	uint32_t rttv = rtt_read_timer_value(RTT);
	uint32_t wait_value = rttv + wait_ms;

//	TRACE("wait: %d %d\r\n", rttv, wait_value);
	cph_millis_delay(1);

	rtt_write_alarm_time(RTT, wait_value);

	cpu_irq_disable();

//	MATRIX->CCFG_SYSIO = 0xffffffff;
//	pio_configure(PIOA, PIO_OUTPUT_0, 0xffffffff, 0);
//	pio_configure(PIOB, PIO_OUTPUT_0, 0xffffffff, 0);
//	pio_configure(PIOC, PIO_OUTPUT_0, 0xffffffff, 0);
//	pmc_disable_all_periph_clk();
//	pmc_switch_mck_to_sclk(PMC_MCKR_PRES_CLK_1);
//	pmc_switch_mainck_to_fastrc(CKGR_MOR_MOSCRCF_4_MHz);
//	pmc_osc_disable_xtal(0);
//	pmc_disable_pllack();
//	pmc_disable_pllbck();
//	pmc_switch_mck_to_mainck(PMC_MCKR_PRES_CLK_1);

	pmc_sleep(SAM_PM_SMODE_WAIT);

	cpu_irq_enable();

	// update our millis counter
	g_cph_millis += wait_ms;
#else
	cph_millis_delay(wait_ms);
#endif

	pio_set_pin_high(DW_WAKEUP_PIO_IDX);
	cph_millis_delay(1);
	pio_set_pin_low(DW_WAKEUP_PIO_IDX);
	cph_millis_delay(50);

	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);
//	cph_deca_init_device();
//	cph_deca_init_network(cph_config->panid, cph_config->shortid);
}

#endif

static int discover(int idx) {
	int result = CPH_OK;

	dwt_setrxaftertxdelay(0);
//	dwt_setrxtimeout(600);
	dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS * 2);

	// Broadcast anchor discovery request
	cph_deca_load_frame((cph_deca_msg_header_t*) &tx_discover_msg, sizeof(tx_discover_msg));
	status_reg = cph_deca_send_response_expected();

	if (status_reg & SYS_STATUS_RXFCG) {
		uint32 frame_len;
		cph_deca_msg_header_t * rx_header;

		// A frame has been received, read it into the local buffer.
		rx_header = cph_deca_read_frame(rx_buffer, &frame_len);
		if (rx_header) {
			// If valid response, send the reply
			if (rx_header->functionCode == FUNC_DISC_REPLY) {

				uint16_t shortid = rx_header->source;

				// Now send the pair response back
				tx_pair_msg.header.dest = shortid;
				cph_deca_load_frame((cph_deca_msg_header_t*) &tx_pair_msg, sizeof(tx_pair_msg));
				cph_deca_send_immediate();

				// Grab the coordinator id
				if (((cph_deca_msg_discover_reply_t*) rx_buffer)->coordid != cph_coordid) {
					cph_coordid = ((cph_deca_msg_discover_reply_t*) rx_buffer)->coordid;
					printf("coordinator discovered at %04X\r\n", cph_coordid);
				}

				// Check for duplicate
				for (int i = 0; i < ANCHORS_MIN; i++) {
					if (anchors[i].shortid == shortid) {
						printf("shortid %04X already exists in anchors[%d]\r\n", shortid, i);
						result = CPH_DUPLICATE;
						break;
					}
				}

				// Not a duplicate so store the shortid
				if (result == CPH_OK) {
					anchors[idx].shortid = shortid;
					anchors[idx].range = 0;
				}

			} else {
				result = CPH_BAD_FRAME;
			}
		} else {
			result = CPH_BAD_LENGTH;
		}
	} else {
		// Clear RX error events in the DW1000 status register.
		TRACE("ERROR: discover status %08X\r\n", status_reg);
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
		result = CPH_ERROR;
	}

	return result;
}

void init_anchors(void) {
	// Init anchors table
	for (int i = 0; i < ANCHORS_MIN; i++) {
		anchors[i].shortid = 0;
		anchors[i].range = 0;
	}
	anchors_status = ANCHORS_MASK;
}

void refresh_anchors(void) {
	int panic_count = 0;

	init_anchors();

	// Discover anchors
	uint32_t anchor_refresh_ts = 0;
	while (anchors_status) {

		panic_count++;
		if (panic_count > 10) {
			TRACE("PANIC! Can't find enough anchors. Sleeping.\r\n");
			sleep_all(5 * 1000);
			panic_count = 0;
		}

		printf("Discovering anchors .. anchors_status:%02X\r\n", anchors_status);

		// Check for refresh of anchors
		uint32_t elapsed = cph_get_millis() - anchor_refresh_ts;
		if (elapsed > ANCHORS_REFRESH_INTERVAL) {
			printf("Anchors discovery timeout.  anchors_status:%02X\r\n", anchors_status);
			init_anchors();
			anchor_refresh_ts = cph_get_millis();
		}

		for (int i = 0; i < ANCHORS_MIN; i++) {
			if (anchors_status & (1 << i)) {
				int result = discover(i);
				if (result == CPH_OK) {
					anchors_status &= (~(1 << i));
					printf("anchor[%d] %04X\r\n", i, anchors[i].shortid);
				}
				deca_sleep(RNG_DELAY_MS);
			}
		}
		//deca_sleep(POLL_DELAY_MS);
		sleep_all(cph_config->sender_period);
	}

	printf("Anchors discovered. Moving to poll.  anchors_status:%02X\r\n", anchors_status);
}

static void send_ranges(int tries) {
	// Only print if the sender period is an odd number .. hackish
	if (cph_config->sender_period & 0x01) {
		printf("%d\t%04X\t", tries, cph_coordid);
		for (int i = 0; i < ANCHORS_MIN; i++) {
			printf("%04X: %3.2f m\t", anchors[i].shortid, anchors[i].range);
		}
		printf("\r\n");
	}

	if (cph_coordid) {
		// Now send the results
		tx_range_results_msg.header.dest = cph_coordid;
		tx_range_results_msg.numranges = ANCHORS_MIN;
		memcpy(&tx_range_results_msg.ranges[0], &anchors[0], sizeof(cph_deca_anchor_range_t) * ANCHORS_MIN);

		cph_deca_load_frame((cph_deca_msg_header_t*) &tx_range_results_msg, sizeof(tx_range_results_msg));
		cph_deca_send_immediate();
	}
}

#if 0
void twr_tag_run(void) {
	uint32_t start_ms, elapsed_ms, wait_ms;

	// Setup DECAWAVE
	cph_deca_init_device();
	cph_deca_init_network(cph_config->panid, cph_config->shortid);

	// Set our short id in common messages
	tx_discover_msg.header.source = cph_config->shortid;
	tx_discover_msg.header.panid = cph_config->panid;

	tx_pair_msg.header.source = cph_config->shortid;
	tx_pair_msg.header.panid = cph_config->panid;

	tx_range_results_msg.header.source = cph_config->shortid;
	tx_range_results_msg.header.panid = cph_config->panid;

	// First, discover anchors
	uint32_t anchor_refresh_ts = 0;
	refresh_anchors();
	anchor_refresh_ts = cph_get_millis();

	// Poll loop
	while (1) {

		int ranges_countdown = MAX_RANGES_BEFORE_POLL_TIMEOUT;
		anchors_status = ANCHORS_MASK;

		start_ms = cph_get_millis();

		while (anchors_status && (--ranges_countdown)) {

			// Check for refresh of anchors
			uint32_t elapsed = cph_get_millis() - anchor_refresh_ts;
			if (elapsed > ANCHORS_REFRESH_INTERVAL) {
				printf("Anchors refresh timeout.  anchors_status:%02X\r\n", anchors_status);
				refresh_anchors();
				anchor_refresh_ts = cph_get_millis();
				// Since we refreshed the anchors, need to range again for ALL anchors during this poll
				anchors_status = ANCHORS_MASK;
			}

			// Range each anchor once during this poll
			for (int i = 0; i < ANCHORS_MIN; i++) {
				if (anchors_status & (1 << i)) {
					anchors[i].range = 0;
					int result = cph_deca_range(&anchors[i], rx_buffer);

					if (result == CPH_OK) {
						anchors_status &= (~(1 << i));
					}

					deca_sleep(RNG_DELAY_MS);
				}
			}

			deca_sleep(RNG_DELAY_MS);
		}

		if (ranges_countdown) {
			send_ranges(MAX_RANGES_BEFORE_POLL_TIMEOUT - ranges_countdown);
		} else {
			printf("ranges_countdown expired!\r\n");
		}

		// Execute a delay between ranging exchanges.
		elapsed_ms = cph_get_millis() - start_ms;
		wait_ms = POLL_DELAY_MS - elapsed_ms;
		if (wait_ms > POLL_DELAY_MS)
			wait_ms = POLL_DELAY_MS;
		deca_sleep(wait_ms);
	}
}

#else


//static void sleep_all(uint32_t wait_ms) {
//	dwt_entersleep();
//
////	rtt_write_alarm_time(RTT, rtt_read_timer_value(RTT) + wait_ms);
////	pmc_sleep(SAM_PM_SMODE_WAIT);
//	cph_millis_delay(wait_ms);
//
//	pio_set_pin_high(DW_WAKEUP_PIO_IDX);
//	cph_millis_delay(1);
//	pio_set_pin_low(DW_WAKEUP_PIO_IDX);
//	cph_millis_delay(5);
//
//	// reset antenna delay, it is not preserved
//	dwt_setrxantennadelay(RX_ANT_DLY);
//	dwt_settxantennadelay(TX_ANT_DLY);
//}



void twr_tag_run(void) {
	uint32_t start_ms, elapsed_ms, wait_ms;
	uint32_t id;

	// Setup interrupt for DW1000 (disable during configuration)
	cph_deca_isr_init();
	cph_deca_isr_disable();

	// Wakeup if necessary
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

	// Setup DECAWAVE
	cph_deca_init_device();
	cph_deca_init_network(cph_config->panid, cph_config->shortid);

	id = dwt_readdevid();
	printf("Device ID: %08X\r\n", id);

	// Set our short id in common messages
	tx_discover_msg.header.source = cph_config->shortid;
	tx_discover_msg.header.panid = cph_config->panid;

	tx_pair_msg.header.source = cph_config->shortid;
	tx_pair_msg.header.panid = cph_config->panid;

	tx_range_results_msg.header.source = cph_config->shortid;
	tx_range_results_msg.header.panid = cph_config->panid;

	// Attach DW interrupt events and callbacks and enable local interrupt pin
	cph_deca_isr_configure();
	cph_deca_isr_enable();

	dwt_setautorxreenable(1);

	// Configure sleep parameters
	dwt_configuresleep(DWT_PRESRV_SLEEP | DWT_CONFIG, DWT_WAKE_WK | DWT_SLP_EN);

	// Setup RTT to count every millisecond
	rtt_init(RTT, 32);
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_ALMIEN);
	pmc_set_fast_startup_input(PMC_FSMR_RTTAL);


	// First, discover anchors
	uint32_t anchor_refresh_ts = 0;
	refresh_anchors();
	anchor_refresh_ts = cph_get_millis();


	// Poll loop
	while (1) {

		int ranges_countdown = MAX_RANGES_BEFORE_POLL_TIMEOUT;
		anchors_status = ANCHORS_MASK;

		start_ms = cph_get_millis();

		while (anchors_status && (--ranges_countdown)) {

			// Check for refresh of anchors
			uint32_t elapsed = cph_get_millis() - anchor_refresh_ts;
			if ((elapsed > ANCHORS_REFRESH_INTERVAL)) {
				printf("Anchors refresh timeout.  anchors_status:%02X\r\n", anchors_status);
				refresh_anchors();
				anchor_refresh_ts = cph_get_millis();
				// Since we refreshed the anchors, need to range again for ALL anchors during this poll
				anchors_status = ANCHORS_MASK;
			}

			// Range each anchor once during this poll
			for (int i = 0; i < ANCHORS_MIN; i++) {
				if (anchors_status & (1 << i)) {
					anchors[i].range = 0;
					int result = cph_deca_range(&anchors[i], rx_buffer);

					if (result == CPH_OK) {
						anchors_status &= (~(1 << i));
					}
					else {
						//TRACE("ERROR: cph_deca_range returned %04X\r\n", result);
					}

					deca_sleep(RNG_DELAY_MS);
				}
			}

			deca_sleep(RNG_DELAY_MS);
		}

		if (ranges_countdown) {
			send_ranges(MAX_RANGES_BEFORE_POLL_TIMEOUT - ranges_countdown);

			// Execute a delay between ranging exchanges.
			elapsed_ms = cph_get_millis() - start_ms;
			wait_ms = cph_config->sender_period - elapsed_ms;
			if (wait_ms > cph_config->sender_period)
				wait_ms = cph_config->sender_period;

//			deca_sleep(wait_ms);
			sleep_all(wait_ms);

		} else {
			TRACE("ranges_countdown expired! sleeping\r\n");
			sleep_all(237);
		}

	}
}

#endif

