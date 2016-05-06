/*
 * twr_anchor.c
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

// Experimental code, work in progress ... We'll get here eventually
//#define EXPERIMENTAL_STATE_MACHINE

#define INTERRUPT_POLLING
//#define STATUS_POLLING

static cph_deca_msg_range_response_t tx_range_response = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		MAC_TAG_ID,		// mac.dest
		MAC_ANCHOR_ID,	// mac.source
		FUNC_RANGE_RESP,		// functionCode
		0x00000000,		// pollRxTs
		0x00000000,		// respTxTs
		0x0000			// mac_cs
		};

static cph_deca_msg_discover_reply_t tx_discover_reply = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		MAC_TAG_ID,		// mac.dest
		MAC_ANCHOR_ID,	// mac.source
		FUNC_DISC_REPLY,	// functionCode
		0x0000,			// coordid
		0x0000			// mac_cs
		};

static cph_deca_msg_coord_announce_t tx_coord_announce = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		0xFFFF,			// mac.dest
		MAC_ANCHOR_ID,	// mac.source
		FUNC_COORD_ANNO,	// functionCode
		0x0000			// mac_cs
		};

static cph_deca_msg_range_result_t tx_range_result = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		MAC_TAG_ID,		// mac.dest
		MAC_ANCHOR_ID,	// mac.source
		FUNC_RANGE_RESU,	// functionCode
		0,				// cph_deca_anchor_range_t
		0x0000			// mac_cs
		};

static cph_deca_msg_survey_request_t tx_survey_request = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		0xFFFF,			// mac.dest
		MAC_ANCHOR_ID,	// mac.source
		FUNC_SURV_REQU,	// functionCode
		0,				// target_short_id
		0,				// reps
		0,				// periodms
		0x0000			// mac_cs
		};

static cph_deca_msg_survey_response_t tx_survey_response = {
MAC_FC,			// mac.ctl - data frame, frame pending, pan id comp, short dest, short source
		0,				// mac.seq
		MAC_PAN_ID,		// mac.panid
		MAC_TAG_ID,		// mac.dest
		MAC_ANCHOR_ID,	// mac.source
		FUNC_SURV_RESP,	// functionCode
		0,				// cph_deca_anchor_range_t
		0,				// error count
		0x0000			// mac_cs
		};

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
static uint8 rx_buffer[CPH_MAX_MSG_SIZE];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
typedef signed long long int64;

static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

/* List of paired tags */
static cph_deca_pair_info_t paired_tags[MAX_TAGS];

#ifdef EXPERIMENTAL_STATE_MACHINE
static bool is_waiting_for_tx_fin = false;
static bool is_response_expected = false;
#endif

/* Coordinator input buffer */
#define COORD_INPUT_MAX 80
static char input_buf[COORD_INPUT_MAX];
static int input_index = 0;
static volatile uint8_t input_ready = 0x00;

/* Declaration of static functions. */
static uint64 get_rx_timestamp_u64(void);
static uint64 get_tx_timestamp_u64(void);

static bool can_respond_to_discover(uint16_t shortid) {
	for (int i = 0; i < MAX_TAGS; i++) {
		if (paired_tags[i].shortid == shortid) {
			uint32_t elapsed = g_cph_millis - paired_tags[i].paired_ts;
			if (elapsed < PAIR_LIFETIME) {
				// Already paired - return no
				return false;
			} else {
				// Paired, but timed out - return yes
				return true;
			}
		}
	}
	// No pair found - return yes
	return true;
}

static bool update_paired_tags(uint16_t shortid) {
	int first_empty = -1;
	int i = 0;

	for (i = 0; i < MAX_TAGS; i++) {
		if (paired_tags[i].shortid == 0 && first_empty == -1)
			first_empty = i;
		if (paired_tags[i].shortid == shortid) {
			// Found tag, update timestamp and return success
			paired_tags[i].paired_ts = g_cph_millis;
			return true;
		}
	}

	// Not found, so try and add it
	if (first_empty == -1) {
		// No empty slot, return fail
		return false;
	}

	paired_tags[first_empty].shortid = shortid;
	paired_tags[first_empty].paired_ts = g_cph_millis;
	return true;

}

static void announce_coord(int repeat) {

	TRACE("announcing coordinator %04X", cph_coordid);
	if (cph_coordid == cph_config->shortid) {
		TRACE(" (me)");
	}
	TRACE("\r\n");

	// Write announce message to frame buffer
	tx_coord_announce.coordid = cph_coordid;
	cph_deca_load_frame(&tx_coord_announce.header, sizeof(tx_coord_announce));

	// Burst out our announcement
	for (int i = 0; i < repeat; i++) {
		cph_deca_send_immediate();
		deca_sleep(10);
	}
}

static double range_with_anchor(uint16_t shortid, uint16_t reps, uint16_t periodms, int * err_count) {
	int status = CPH_OK;
	double accum;
	int count;

	cph_deca_anchor_range_t anchor;

	anchor.shortid = shortid;

	TRACE("RANGING with %04X\r\n", anchor.shortid);

	// Go back to idle
	dwt_forcetrxoff();

	accum = 0;
	count = 0;
	*err_count = 0;

	for (int i = 0; i < reps; i++) {
		anchor.range = 0;
		if ((status = cph_deca_range(&anchor, rx_buffer)) == CPH_OK) {
			TRACE("%02d Range: %3.2fm\r\n", count, anchor.range);
			accum += anchor.range;
			count++;
		} else {
			TRACE("RANGE ERROR!  %02X\r\n", status);
			(*err_count)++;
		}
		cph_millis_delay(periodms);
	}

	return accum / count;
}

#ifdef EXPERIMENTAL_STATE_MACHINE

static void start_rx(void) {
	is_waiting_for_tx_fin = false;
	is_response_expected = false;
	dwt_setrxtimeout(0);
	dwt_rxenable(0);
}

static int start_tx(uint8_t mode) {
	int result = dwt_starttx(mode);
	if (result == DWT_SUCCESS) {
		is_waiting_for_tx_fin = true;
		if (mode & DWT_RESPONSE_EXPECTED) {
			is_response_expected = true;
		} else {
			is_response_expected = false;
		}
	}
	return result;
}

static void handle_event(const cph_deca_event_t * event) {

//	TRACE("%08X %02X\r\n", event->info.status, event->info.event);

	if (event->status == CPH_EVENT_ERR) {
		TRACE("[ERR] %02X %08X\r\n", event->status, event->info.status);
		start_rx();
	}

	else if (event->status == CPH_EVENT_TX) {
		is_waiting_for_tx_fin = false;
		if (is_response_expected == false) {
			dwt_setrxtimeout(0);
			dwt_rxenable(0);
		}
	}

	else if (is_waiting_for_tx_fin == true) {
		TRACE("[ERR] Event status %02X rcvd while waiting for tx fin\r\n", event->status);
		start_rx();
	}

	else if (event->status == CPH_EVENT_RX) {

		cph_deca_msg_header_t * rx_header;
		rx_header = (cph_deca_msg_header_t *) event->data;

		if (rx_header->functionCode == FUNC_DISC_ANNO) {
			if (can_respond_to_discover(rx_header->source)) {
				/* Write and send the announce message. */
				tx_discover_reply.coordid = cph_coordid;
				tx_discover_reply.header.dest = rx_header->source;
				cph_deca_load_frame(&tx_discover_reply.header, sizeof(tx_discover_reply));
				start_tx(DWT_START_TX_IMMEDIATE);
			} else {
				TRACE("ignoring pair with %04X\r\n", (rx_header->source));
				start_rx();
			}
		}

		else if (rx_header->functionCode == FUNC_PAIR_RESP) {
			//TODO: Record the pairing details and check them when receiving a discover request
			//      For now, nothing to do
			if (update_paired_tags(rx_header->source)) {
				TRACE("paired with %04X\r\n", (rx_header->source));
			} else {
				TRACE("failed to pair with %04X\r\n", (rx_header->source));
			}
			start_rx();
		}

		else if (rx_header->functionCode == FUNC_RANGE_POLL) {
			uint32 resp_tx_time;

			/* Retrieve poll reception timestamp. */
			poll_rx_ts = event->timestamp;

			/* Compute final message transmission time. See NOTE 7 below. */
			resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
			dwt_setdelayedtrxtime(resp_tx_time);

			/* Set expected delay and timeout for final message reception. */
			dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
			dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

			/* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
			resp_tx_ts = (((uint64) (resp_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;

			/* Write all timestamps in the final message. See NOTE 8 below. */
			tx_range_response.pollRxTs = poll_rx_ts;
			tx_range_response.responseTxTs = resp_tx_ts;

			/* Send the response message */
			tx_range_response.header.dest = rx_header->source;
			cph_deca_load_frame(&tx_range_response.header, sizeof(tx_range_response));
			int result = start_tx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
			if (result != DWT_SUCCESS) {
				TRACE("ERROR: dwt_starttx response returned %d - too late!\r\n", result);
				start_rx();
			}
		}

		else if (rx_header->functionCode == FUNC_RANGE_FINA) {

			uint64 final_rx_ts;
			uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
			uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
			double Ra, Rb, Da, Db;
			double distance, tof;
			int64 tof_dtu;

			// Retrieve response transmission and final reception timestamps.
			resp_tx_ts = get_tx_timestamp_u64();	//ERIC: Should pull this from final message
			final_rx_ts = event->timestamp;

			// Get timestamps embedded in the final message.
			poll_tx_ts = ((cph_deca_msg_range_final_t*) rx_header)->pollTxTs;
			resp_rx_ts = ((cph_deca_msg_range_final_t*) rx_header)->responseRxTs;
			final_tx_ts = ((cph_deca_msg_range_final_t*) rx_header)->finalTxTs;

			// Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped.
			poll_rx_ts_32 = (uint32) poll_rx_ts;
			resp_tx_ts_32 = (uint32) resp_tx_ts;
			final_rx_ts_32 = (uint32) final_rx_ts;
			Ra = (double) (resp_rx_ts - poll_tx_ts);
			Rb = (double) (final_rx_ts_32 - resp_tx_ts_32);
			Da = (double) (final_tx_ts - resp_rx_ts);
			Db = (double) (resp_tx_ts_32 - poll_rx_ts_32);
			tof_dtu = (int64) ((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

			tof = tof_dtu * DWT_TIME_UNITS;
			distance = tof * SPEED_OF_LIGHT;

			// Send result back to tag
			tx_range_result.header.dest = rx_header->source;
			tx_range_result.range.shortid = cph_config->shortid;
			tx_range_result.range.range = distance;
			cph_deca_load_frame(&tx_range_result.header, sizeof(tx_range_result));
			start_tx(DWT_START_TX_IMMEDIATE);

			TRACE("%04X DIST: %3.2f m\r\n", rx_header->source, distance);
		}

		else {
			TRACE("[RCV] ");
			for (int i = 0; i < event->info.datalength; i++)
				TRACE("%02X ", event->data[i]);
			TRACE("\r\n");

			start_rx();
		}
	}
}
#endif

static void stdio_data_ready_handler(void)
{
	//TODO: Move all of line-input stuff to cph_stdio

	if (input_index >= COORD_INPUT_MAX)
		return;

	uint8_t prev_index = input_index;

	int count = cph_stdio_read_buf(&input_buf[input_index], COORD_INPUT_MAX - input_index);
	input_index += count;

	for (int i=prev_index; i < input_index;i++) {
		if (input_buf[i] == '\r')
		{
			input_ready = 0xFF;
			return;
		}
	}

	// If we hit the limit and reached this point (input_ready is false), clear the
	// buffer and start over.
	if (input_index >= COORD_INPUT_MAX) {
		TRACE("[OVF]\r\n");
		input_index = 0;
		input_ready = 0x00;
		memset(input_buf, 0, COORD_INPUT_MAX);
	}
}

static bool stdio_line_handler(void) {
	bool result = true;
	int shortid_a, shortid_b;
	int x, y;

	TRACE("Command: %s\r\n", input_buf);

	if (input_buf[0] == 'r') {
		if (sscanf(&input_buf[2], "%x %x %d %d", &shortid_a, &shortid_b, &x, &y) != 4) {
			TRACE("BAD FORMAT\r\n");
		} else {
			// send message
			dwt_forcetrxoff();
			tx_survey_request.header.dest = shortid_a;
			tx_survey_request.target_short_id = shortid_b;
			tx_survey_request.reps = x;
			tx_survey_request.periodms = y;
			cph_deca_load_frame(&tx_survey_request.header, sizeof(tx_survey_request));
			cph_deca_send_immediate();
		}
	}
	else {
		result = false;
	}

	input_index = 0;
	input_ready = 0x00;
	memset(input_buf, 0, COORD_INPUT_MAX);

	return result;
}

void twr_anchor_run(void) {
	uint32_t announce_coord_ts = 0;
	uint32_t elapsed = 0;

	// Attach stdio handler
	memset(input_buf, 0, COORD_INPUT_MAX);
	cph_stdio_set_datready_cb(stdio_data_ready_handler);

	// Setup interrupt for DW1000 (disable during configuration)
	cph_deca_isr_init();
	cph_deca_isr_disable();

	// Setup DW1000
	cph_deca_init_device();
	cph_deca_init_network(cph_config->panid, cph_config->shortid);

	// Init list of paired tags
	memset(paired_tags, 0, sizeof(cph_deca_pair_info_t) * MAX_TAGS);

	// Set our shortid in common messages
	tx_range_response.header.source = cph_config->shortid;
	tx_range_response.header.panid = cph_config->panid;

	tx_range_result.header.source = cph_config->shortid;
	tx_range_result.header.panid = cph_config->panid;

	tx_discover_reply.header.source = cph_config->shortid;
	tx_discover_reply.header.panid = cph_config->panid;

	tx_coord_announce.header.source = cph_config->shortid;
	tx_coord_announce.header.panid = cph_config->panid;

	tx_survey_request.header.source = cph_config->shortid;
	tx_survey_request.header.panid = cph_config->panid;

	tx_survey_response.header.source = cph_config->shortid;
	tx_survey_response.header.panid = cph_config->panid;


#ifdef EXPERIMENTAL_STATE_MACHINE

	// Attach DW interrupt events and callbacks and enable local interrupt pin
	cph_deca_isr_configure();
	cph_deca_isr_enable();

	dwt_setrxtimeout(0);
	dwt_setautorxreenable(1);
	dwt_rxenable(0);

	cph_deca_event_t event;

	while (1) {
		if (cph_deca_get_event(&event)) {
//			TRACE("[QUE] %02X\r\n", event.status);
			handle_event(&event);
		}
	}



#elif defined(INTERRUPT_POLLING)

	// Attach DW interrupt events and callbacks and enable local interrupt pin
	cph_deca_isr_configure();
	cph_deca_isr_enable();

	dwt_setautorxreenable(1);


	// Burst announce ourselves if we're the coordinator
	if (cph_config->mode == CPH_MODE_COORD) {
		TRACE("Bursting coordinator announcement...\r\n");
		cph_coordid = cph_config->shortid;
		announce_coord(COORD_ANNOUNCE_START_BURST);
		announce_coord_ts = cph_get_millis();
	}

	TRACE("Beginning loop...\r\n");

	/* Loop forever responding to ranging requests. */
	while (1) {

		if (cph_coordid) {
			elapsed = cph_get_millis() - announce_coord_ts;
			if (elapsed > COORD_ANNOUNCE_INTERVAL) {
				announce_coord(1);
				announce_coord_ts = cph_get_millis();
			}
		}

		/* Activate reception immediately. */
		dwt_setrxtimeout(0);
		dwt_rxenable(0);

		status_reg = cph_deca_wait_for_rx_finished_signal(DEFAULT_RX_TIMEOUT * 2, &input_ready);

		if (input_ready == 0xFF) {
			stdio_line_handler();
			//TODO: Move all of line-input stuff to cph_stdio
			continue;
		}

		if (status_reg & SYS_STATUS_RXFCG) {
			uint32 frame_len;
			cph_deca_msg_header_t * rx_header;

			rx_header = cph_deca_read_frame(rx_buffer, &frame_len);


//			TRACE("[RCV] ");
//			for (int i = 0; i < frame_len; i++)
//				TRACE("%02X ", rx_buffer[i]);
//			TRACE("\r\n");
//			continue;

			// If we're the coordinator, only accept range reports and commands
			if (cph_config->mode == CPH_MODE_COORD) {
				if (rx_header->functionCode != FUNC_RANGE_REPO && rx_header->functionCode != FUNC_SURV_REQU && rx_header->functionCode != FUNC_SURV_RESP && rx_header->functionCode != FUNC_COORD_ANNO)
				continue;
			}

			// Look for Poll message
			if (rx_header->functionCode == FUNC_RANGE_POLL) {
				uint32 resp_tx_time;

				/* Retrieve poll reception timestamp. */
				poll_rx_ts = get_rx_timestamp_u64();

				/* Compute final message transmission time. See NOTE 7 below. */
				resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(resp_tx_time);

				/* Set expected delay and timeout for final message reception. */
				dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
				dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

				/* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
				resp_tx_ts = (((uint64) (resp_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;

				/* Write all timestamps in the final message. See NOTE 8 below. */
				tx_range_response.pollRxTs = poll_rx_ts;
				tx_range_response.responseTxTs = resp_tx_ts;

				/* Send the response message */
				tx_range_response.header.dest = rx_header->source;
				cph_deca_load_frame(&tx_range_response.header, sizeof(tx_range_response));
				cph_deca_send_delayed();

			} else if (rx_header->functionCode == FUNC_RANGE_FINA) {

				uint64 final_rx_ts;
				uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
				uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
				double Ra, Rb, Da, Db;
				double distance, tof;
				int64 tof_dtu;

				// Retrieve response transmission and final reception timestamps.
				resp_tx_ts = get_tx_timestamp_u64();//ERIC: Should pull this from final message
				final_rx_ts = get_rx_timestamp_u64();

				// Get timestamps embedded in the final message.
				poll_tx_ts = ((cph_deca_msg_range_final_t*) rx_header)->pollTxTs;
				resp_rx_ts = ((cph_deca_msg_range_final_t*) rx_header)->responseRxTs;
				final_tx_ts = ((cph_deca_msg_range_final_t*) rx_header)->finalTxTs;

				// Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped.
				poll_rx_ts_32 = (uint32) poll_rx_ts;
				resp_tx_ts_32 = (uint32) resp_tx_ts;
				final_rx_ts_32 = (uint32) final_rx_ts;
				Ra = (double) (resp_rx_ts - poll_tx_ts);
				Rb = (double) (final_rx_ts_32 - resp_tx_ts_32);
				Da = (double) (final_tx_ts - resp_rx_ts);
				Db = (double) (resp_tx_ts_32 - poll_rx_ts_32);
				tof_dtu = (int64) ((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

				tof = tof_dtu * DWT_TIME_UNITS;
				distance = tof * SPEED_OF_LIGHT;

				// Send result back to tag
				tx_range_result.header.dest = rx_header->source;
				tx_range_result.range.shortid = cph_config->shortid;
				tx_range_result.range.range = distance;
				cph_deca_load_frame(&tx_range_result.header, sizeof(tx_range_result));
				cph_deca_send_immediate();

				if (cph_config->mode == CPH_MODE_COORD) {
					TRACE("%04X DIST: %3.2f m\r\n", rx_header->source, distance);
				}

			} else if (rx_header->functionCode == FUNC_DISC_ANNO) {

				if (can_respond_to_discover(rx_header->source)) {
					/* Write and send the announce message. */
					tx_discover_reply.coordid = cph_coordid;
					tx_discover_reply.header.dest = rx_header->source;
					cph_deca_load_frame(&tx_discover_reply.header, sizeof(tx_discover_reply));
					cph_deca_send_immediate();
				} else {
					TRACE("ignoring pair with %04X\r\n", (rx_header->source));
				}

			} else if (rx_header->functionCode == FUNC_PAIR_RESP) {
				//TODO: Record the pairing details and check them when receiving a discover request
				//      For now, nothing to do
				if (update_paired_tags(rx_header->source)) {
					TRACE("paired with %04X\r\n", (rx_header->source));
				} else {
					TRACE("failed to pair with %04X\r\n", (rx_header->source));
				}

			} else if (rx_header->functionCode == FUNC_COORD_ANNO) {
				uint16_t id = ((cph_deca_msg_coord_announce_t*) rx_buffer)->coordid;
				if (id != cph_coordid) {
					cph_coordid = id;
					if (cph_coordid == cph_config->shortid) {
						TRACE("becoming coord\r\n");
						cph_config->mode = CPH_MODE_COORD;
					} else {
						if (cph_config->mode == CPH_MODE_COORD) {
							TRACE("giving coord to %04X\r\n", cph_coordid);
						} else {
							TRACE("recognizing coord as %04X\r\n", cph_coordid);
						}
						cph_config->mode = CPH_MODE_ANCHOR;
					}
				}
				// If I'm the coordinator, spit out the announcement - this acts as the anchor report
				if (cph_config->mode == CPH_MODE_COORD) {
					TRACE("* A %04X %04X\r\n", rx_header->source, id);
				}

			} else if (rx_header->functionCode == FUNC_RANGE_REPO) {
				cph_deca_msg_range_report_t * results = ((cph_deca_msg_range_report_t*) rx_buffer);
				TRACE("* %04X", rx_header->source);
				for (int i = 0; i < results->numranges; i++) {
					TRACE(" %04X:%3.2f", results->ranges[i].shortid, results->ranges[i].range);
				}
				TRACE("\r\n");

			} else if (rx_header->functionCode == FUNC_SURV_REQU) {
				cph_deca_msg_survey_request_t * req = ((cph_deca_msg_survey_request_t*) rx_buffer);
				uint16_t sourceid = req->header.source;
				uint16_t targetid = req->target_short_id;
				int err_count = 0;

				TRACE("Survey req from %04X: %04X %d %d\r\n", req->header.source, req->target_short_id, req->reps, req->periodms);
				double dist = range_with_anchor(req->target_short_id, req->reps, req->periodms, &err_count);
				TRACE("AVERAGE RANGE ====> %3.2fm\r\n", dist);

				tx_survey_response.header.dest = sourceid;
				tx_survey_response.range.shortid = targetid;
				tx_survey_response.range.range = dist;
				tx_survey_response.error_count = err_count;
				cph_deca_load_frame(&tx_survey_response.header, sizeof(tx_survey_response));
				cph_deca_send_immediate();

			} else if (rx_header->functionCode == FUNC_SURV_RESP) {
				cph_deca_msg_survey_response_t * resp = ((cph_deca_msg_survey_response_t*) rx_buffer);
				TRACE("* S %04X %04X %3.2f %d\r\n", resp->header.source, resp->range.shortid, resp->range.range, resp->error_count);

			} else {
				TRACE("ERROR: unknown function code - data %02X: ", rx_header->functionCode);
				for (int i = 0; i < frame_len; i++)
				TRACE("%02X ", rx_buffer[i]);
				TRACE("\r\n");
			}
		} else {

			// Ignore frame rejections and timeouts
//			uint32_t test = status_reg & (~(SYS_STATUS_AFFREJ | SYS_STATUS_RXRFTO));
//			if (test & SYS_STATUS_ALL_RX_ERR)
			{
				TRACE("ERROR: dwt_rxenable %08X\r\n", status_reg);
				/* Clear RX error events in the DW1000 status register. */
				//dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
				dwt_write32bitreg(SYS_STATUS_ID, status_reg);
			}
		}
	}
#elif defined(STATUS_POLLING)
	/* Loop forever responding to ranging requests. */
	while (1) {

		if (cph_coordid) {
			elapsed = cph_get_millis() - announce_coord_ts;
			if (elapsed > COORD_ANNOUNCE_INTERVAL) {
				announce_coord(1);
				announce_coord_ts = cph_get_millis();
			}
		}

		/* Activate reception immediately. */
		dwt_setrxtimeout(0);
		dwt_rxenable(0);

		while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {
		};

		if (status_reg & SYS_STATUS_RXFCG) {
			uint32 frame_len;
			cph_deca_msg_header_t * rx_header;

			rx_header = cph_deca_read_frame(rx_buffer, &frame_len);

			// If we're the coordinator, only accept range reports and commands
			if (cph_config->mode == CPH_MODE_COORD) {
				if (rx_header->functionCode != FUNC_RANGE_REPO && rx_header->functionCode != FUNC_SURV_REQU && rx_header->functionCode != FUNC_SURV_RESP && rx_header->functionCode != FUNC_COORD_ANNO)
				continue;
			}

			// Look for Poll message
			if (rx_header->functionCode == FUNC_RANGE_POLL) {
				uint32 resp_tx_time;

				/* Retrieve poll reception timestamp. */
				poll_rx_ts = get_rx_timestamp_u64();

				/* Compute final message transmission time. See NOTE 7 below. */
				resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(resp_tx_time);

				/* Set expected delay and timeout for final message reception. */
				dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
				dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

				/* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
				resp_tx_ts = (((uint64) (resp_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;

				/* Write all timestamps in the final message. See NOTE 8 below. */
				tx_range_response.pollRxTs = poll_rx_ts;
				tx_range_response.responseTxTs = resp_tx_ts;

				/* Send the response message */
				tx_range_response.header.dest = rx_header->source;
				cph_deca_load_frame(&tx_range_response.header, sizeof(tx_range_response));
				cph_deca_send_delayed();

			} else if (rx_header->functionCode == FUNC_RANGE_FINA) {

				uint64 final_rx_ts;
				uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
				uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
				double Ra, Rb, Da, Db;
				double distance, tof;
				int64 tof_dtu;

				// Retrieve response transmission and final reception timestamps.
				resp_tx_ts = get_tx_timestamp_u64();//ERIC: Should pull this from final message
				final_rx_ts = get_rx_timestamp_u64();

				// Get timestamps embedded in the final message.
				poll_tx_ts = ((cph_deca_msg_range_final_t*) rx_header)->pollTxTs;
				resp_rx_ts = ((cph_deca_msg_range_final_t*) rx_header)->responseRxTs;
				final_tx_ts = ((cph_deca_msg_range_final_t*) rx_header)->finalTxTs;

				// Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped.
				poll_rx_ts_32 = (uint32) poll_rx_ts;
				resp_tx_ts_32 = (uint32) resp_tx_ts;
				final_rx_ts_32 = (uint32) final_rx_ts;
				Ra = (double) (resp_rx_ts - poll_tx_ts);
				Rb = (double) (final_rx_ts_32 - resp_tx_ts_32);
				Da = (double) (final_tx_ts - resp_rx_ts);
				Db = (double) (resp_tx_ts_32 - poll_rx_ts_32);
				tof_dtu = (int64) ((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

				tof = tof_dtu * DWT_TIME_UNITS;
				distance = tof * SPEED_OF_LIGHT;

				// Send result back to tag
				tx_range_result.header.dest = rx_header->source;
				tx_range_result.range.shortid = cph_config->shortid;
				tx_range_result.range.range = distance;
				cph_deca_load_frame(&tx_range_result.header, sizeof(tx_range_result));
				cph_deca_send_immediate();

				if (cph_config->mode == CPH_MODE_COORD) {
					TRACE("%04X DIST: %3.2f m\r\n", rx_header->source, distance);
				}

			} else if (rx_header->functionCode == FUNC_DISC_ANNO) {

				if (can_respond_to_discover(rx_header->source)) {
					/* Write and send the announce message. */
					tx_discover_reply.coordid = cph_coordid;
					tx_discover_reply.header.dest = rx_header->source;
					cph_deca_load_frame(&tx_discover_reply.header, sizeof(tx_discover_reply));
					cph_deca_send_immediate();
				} else {
					TRACE("ignoring pair with %04X\r\n", (rx_header->source));
				}

			} else if (rx_header->functionCode == FUNC_PAIR_RESP) {
				//TODO: Record the pairing details and check them when receiving a discover request
				//      For now, nothing to do
				if (update_paired_tags(rx_header->source)) {
					TRACE("paired with %04X\r\n", (rx_header->source));
				} else {
					TRACE("failed to pair with %04X\r\n", (rx_header->source));
				}

			} else if (rx_header->functionCode == FUNC_COORD_ANNO) {
				uint16_t id = ((cph_deca_msg_coord_announce_t*) rx_buffer)->coordid;
				if (id != cph_coordid) {
					cph_coordid = id;
					if (cph_coordid == cph_config->shortid) {
						TRACE("becoming coord\r\n");
						cph_config->mode = CPH_MODE_COORD;
					} else {
						if (cph_config->mode == CPH_MODE_COORD) {
							TRACE("giving coord to %04X\r\n", cph_coordid);
						} else {
							TRACE("recognizing coord as %04X\r\n", cph_coordid);
						}
						cph_config->mode = CPH_MODE_ANCHOR;
					}
				}
				// If I'm the coordinator, spit out the announcement - this acts as the anchor report
				if (cph_config->mode == CPH_MODE_COORD) {
					TRACE("* A %04X %04X\r\n", rx_header->source, id);
				}

			} else if (rx_header->functionCode == FUNC_RANGE_REPO) {
				cph_deca_msg_range_report_t * results = ((cph_deca_msg_range_report_t*) rx_buffer);
				TRACE("* %04X", rx_header->source);
				for (int i = 0; i < results->numranges; i++) {
					TRACE(" %04X:%3.2f", results->ranges[i].shortid, results->ranges[i].range);
				}
				TRACE("\r\n");

			} else if (rx_header->functionCode == FUNC_SURV_REQU) {
				cph_deca_msg_survey_request_t * req = ((cph_deca_msg_survey_request_t*) rx_buffer);
				uint16_t sourceid = req->header.source;
				uint16_t targetid = req->target_short_id;
				int err_count = 0;

				TRACE("Survey req from %04X: %04X %d %d\r\n", req->header.source, req->target_short_id, req->reps, req->periodms);
				double dist = range_with_anchor(req->target_short_id, req->reps, req->periodms, &err_count);
				TRACE("AVERAGE RANGE ====> %3.2fm\r\n", dist);

				tx_survey_response.header.dest = sourceid;
				tx_survey_response.range.shortid = targetid;
				tx_survey_response.range.range = dist;
				tx_survey_response.error_count = err_count;
				cph_deca_load_frame(&tx_survey_response.header, sizeof(tx_survey_response));
				cph_deca_send_immediate();

			} else if (rx_header->functionCode == FUNC_SURV_RESP) {
				cph_deca_msg_survey_response_t * resp = ((cph_deca_msg_survey_response_t*) rx_buffer);
				TRACE("* S %04X %04X %3.2f %d\r\n", resp->header.source, resp->range.shortid, resp->range.range, resp->error_count);

			} else {
				TRACE("ERROR: unknown function code - data %02X: ", rx_header->functionCode);
				for (int i = 0; i < frame_len; i++)
				TRACE("%02X ", rx_buffer[i]);
				TRACE("\r\n");
			}
		} else {

			// Ignore frame rejections and timeouts
//			uint32_t test = status_reg & (~(SYS_STATUS_AFFREJ | SYS_STATUS_RXRFTO));
//			if (test & SYS_STATUS_ALL_RX_ERR)
			{
				TRACE("ERROR: dwt_rxenable %08X\r\n", status_reg);
				/* Clear RX error events in the DW1000 status register. */
				dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
			}
		}
	}
#else
#error Pick a polling method: INTERRUPT or STATUS
#endif
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void) {
	uint8 ts_tab[5];
	uint64 ts = 0;
	int i;
	dwt_readrxtimestamp(ts_tab);
	for (i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void) {
	uint8 ts_tab[5];
	uint64 ts = 0;
	int i;
	dwt_readtxtimestamp(ts_tab);
	for (i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}
