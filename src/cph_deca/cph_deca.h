/*
 * cph_deca.h
 *
 *  Created on: Jan 11, 2016
 *      Author: ericrudisill
 */

#ifndef SRC_CPH_DECA_CPH_DECA_H_
#define SRC_CPH_DECA_CPH_DECA_H_

#include <cph_deca_port.h>
#include <deca_regs.h>
#include <deca_device_api.h>

// 								 'C''P'
#define MAC_PAN_ID				0x4350
#define MAC_ANCHOR_ID			0x4157
#define MAC_TAG_ID				0x4556
#define MAC_FC					0x8841
#define MAC_FC_ACK				0x8861
#define MAC_SHORT				0x1234


void twr_anchor_run(void);
void twr_tag_run(void);
void listener_run(void);
void sender_run(void);


/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 	5

/* Inter-poll delay period, in milliseconds */
#define POLL_DELAY_MS 	200

/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436



/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
//#define POLL_TX_TO_RESP_RX_DLY_UUS 150
#define POLL_TX_TO_RESP_RX_DLY_UUS 100
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
//#define RESP_RX_TO_FINAL_TX_DLY_UUS		3100
#define RESP_RX_TO_FINAL_TX_DLY_UUS		3500
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 2700

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
//#define POLL_RX_TO_RESP_TX_DLY_UUS		2600
#define POLL_RX_TO_RESP_TX_DLY_UUS		3000
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS		500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS			3300


/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547


// Min Number of anchors to range with - if this changes, so should ANCHORS_MASK
//#define ANCHORS_MIN		3
#define ANCHORS_MIN		1

// Used for tracking status of anchor ids (by bitmask) during discovery and poll
//#define ANCHORS_MASK	0x07
#define ANCHORS_MASK	0x01

// Anchor refresh interval
#define ANCHORS_REFRESH_INTERVAL	10000

// Coord announce startup burst repeat count
#define COORD_ANNOUNCE_START_BURST	10

// Coord announce period in ms
#define COORD_ANNOUNCE_INTERVAL		7000

// Max ranges before poll timeout - keeps from blasting radio when an anchor is not responding
#define MAX_RANGES_BEFORE_POLL_TIMEOUT	5

// Max number of tags to pair with
#define MAX_TAGS		32

// Lifetime of tag pairing
#define PAIR_LIFETIME	5000

// Delay to start listening after discover
#define DISCOVER_TX_TO_ANNOUNCE_RX_DELAY_UUS	400
#define DISCOVER_RX_TO_ANNOUNCE_TX_DELAY_UUS	460

enum {
	CPH_OK = 0,
	CPH_ERROR,
	CPH_BAD_FRAME,
	CPH_BAD_LENGTH,
	CPH_DUPLICATE
};

enum {
	CPH_MODE_ANCHOR = 0x01,
	CPH_MODE_TAG = 0x02,
	CPH_MODE_LISTENER = 0x03,
	CPH_MODE_SENDER = 0x04,
	CPH_MODE_COORD = 0x80
};

#define	FUNC_RANGE_POLL				0xA0
#define FUNC_RANGE_RESP				0xA1
#define FUNC_RANGE_FINA				0xA2
#define FUNC_RANGE_RESU				0xA3
#define FUNC_RANGE_REPO				0xA4

#define FUNC_RANGE_BURST			0xAF

#define FUNC_DISC_ANNO				0xB2
#define FUNC_DISC_REPLY				0xB3
#define FUNC_PAIR_RESP				0xB4

#define FUNC_COORD_ANNO				0xC5

#define FUNC_SURV_REQU				0x01
#define FUNC_SURV_RESP				0x21

#define CPH_MAX_MSG_SIZE		128

#define CPH_MAX_EVENTS			10
#define CPH_EVENT_RCV		0xff
#define CPH_EVENT_ERR		0xee

#define PACKED	__attribute__((packed))

typedef struct PACKED {
	uint16_t ctl;
	uint8_t seq;
	uint16_t panid;
	uint16_t dest;
	uint16_t source;
	uint8_t functionCode;
} cph_deca_msg_header_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	uint16_t mac_cs;
} cph_deca_msg_range_request_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	uint32_t pollRxTs;
	uint32_t responseTxTs;
	uint16_t mac_cs;
} cph_deca_msg_range_response_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	uint32_t pollTxTs;
	uint32_t responseTxTs;
	uint32_t responseRxTs;
	uint32_t finalTxTs;
	uint16_t mac_cs;
} cph_deca_msg_range_final_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	uint16_t mac_cs;
} cph_deca_msg_discover_announce_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	uint16_t coordid;
	uint16_t mac_cs;
} cph_deca_msg_discover_reply_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	uint16_t mac_cs;
} cph_deca_msg_pair_response_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	uint16_t coordid;
	uint16_t mac_cs;
} cph_deca_msg_coord_announce_t;

typedef struct PACKED {
	uint16_t shortid;
	double range;
} cph_deca_anchor_range_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	cph_deca_anchor_range_t range;
	uint16_t mac_cs;
} cph_deca_msg_range_result_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	uint8_t numranges;
	cph_deca_anchor_range_t ranges[ANCHORS_MIN];		//TODO: Make this dynamic
	uint16_t mac_fs;
} cph_deca_msg_range_report_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	uint16_t target_short_id;
	uint16_t reps;
	uint16_t periodms;
	uint16_t mac_cs;
} cph_deca_msg_survey_request_t;

typedef struct PACKED {
	cph_deca_msg_header_t header;
	cph_deca_anchor_range_t range;
	uint16_t error_count;
	uint16_t mac_cs;
} cph_deca_msg_survey_response_t;

typedef struct PACKED {
	uint16_t shortid;
	uint32_t paired_ts;
} cph_deca_pair_info_t;



typedef struct PACKED {
	dwt_callback_data_t info;
	uint8_t status;
	uint8_t data[CPH_MAX_MSG_SIZE];
} cph_deca_event_t;



inline uint32_t cph_deca_wait_for_tx_finished(void) {
	uint32_t status_reg;
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & SYS_STATUS_TXFRS)) {
	};
	return status_reg;
}

inline uint32_t cph_deca_wait_for_rx_finished(void) {
	uint32_t status_reg;
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {
	};
	return status_reg;
}


void cph_deca_load_frame(cph_deca_msg_header_t * hdr, uint16_t size);
cph_deca_msg_header_t * cph_deca_read_frame(uint8_t * rx_buffer, uint32_t *frame_len);
uint32_t cph_deca_send_immediate();
uint32_t cph_deca_send_delayed();
uint32_t cph_deca_send_delayed_response_expected();
uint32_t cph_deca_send_response_expected();
void cph_deca_init_device();
void cph_deca_init_network(uint16_t panid, uint16_t shortid);
void cph_deca_isr_handler(uint32_t id, uint32_t mask);
void cph_deca_isr_init(void);
void cph_deca_isr_configure(void) ;
void cph_deca_rxcallback(const dwt_callback_data_t *rxd);
bool cph_deca_get_event(cph_deca_event_t * event);

#endif /* SRC_CPH_DECA_CPH_DECA_H_ */
