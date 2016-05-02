/*
 * cs_listener.c
 *
 *  Created on: Apr 5, 2016
 *      Author: Will
 */

#include<cph.h>
#include<inttypes.h>

#define MAX_BUFF_SIZE	64
#define CHIRP_PERIOD 5000
#define STATUS_EMPTY 0x00
#define STATUS_RCV 0x01
#define STATUS_ERR 0x02
#define STATUS_BAD 0x03

#define SAMPLE_SIZE 30
#define MATH 0
#define FIR 0x01
#define KALMAN 0x02
#define LINEAR 0x03
#define AWAKE_CHIRP 0

static uint8 rx_buffer[MAX_BUFF_SIZE];
static volatile uint32 frame_len;
static volatile uint8 irq_status = STATUS_EMPTY;

double fir_array[SAMPLE_SIZE] = {0};
int fir_index = 0;
double do_math(double base_val, int mode) {
	double mathed_val = 0;
	int count = 0;
	switch(mode) {
	case FIR:
		fir_array[fir_index] = base_val;
		fir_index = (fir_index + 1) % SAMPLE_SIZE;
		for(int i = 0; i < SAMPLE_SIZE; i++) {
			if(!(fir_array[i] == 0)) {
				mathed_val += fir_array[i];
				count++;
			}
		}
		mathed_val = mathed_val / count;
		break;
	case KALMAN:
		break;
	case LINEAR:
		break;
	}
	return mathed_val;
}

static void irq_handler(uint32 id, uint32 mask) {
	do {
		dwt_isr();
	} while (cph_deca_isr_is_detected() == 1);
}

static void irq_init(void) {
	pio_configure_pin(DW_IRQ_IDX, DW_IRQ_FLAGS);
	pio_pull_down(DW_IRQ_PIO, DW_IRQ_MASK, true);
	pio_handler_set(DW_IRQ_PIO, DW_IRQ_PIO_ID, DW_IRQ_MASK, DW_IRQ_ATTR, irq_handler);
	pio_enable_interrupt(DW_IRQ_PIO, DW_IRQ_MASK);

	pio_handler_set_priority(DW_IRQ_PIO, DW_IRQ_IRQ, 0);

	pmc_enable_periph_clk(DW_IRQ_PIO_ID);
}

static void rxcallback(const dwt_callback_data_t *rxd) {
	if (rxd->event == DWT_SIG_RX_OKAY) {
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
		if (rxd->datalength <= MAX_BUFF_SIZE) {
			dwt_readrxdata(rx_buffer, rxd->datalength, 0);
			irq_status = STATUS_RCV;
			frame_len = rxd->datalength;}
		else {
			irq_status = STATUS_ERR;
			frame_len = rxd->datalength;
		}}
	else {
		dwt_readrxdata(rx_buffer, 20, 0);
		frame_len = 20;
		irq_status = STATUS_ERR;
	}
}

void print_buffer() {
	TRACE("[RCV]: ");
	for(int i = 0; i < frame_len; i++)
	{
		TRACE("%02X", rx_buffer[i]);
	}
	TRACE("\r\n");
}

uint64_t get_rx_timestamp(void) {
	uint8_t ts_tab[5];
	uint64_t ts = 0;
	int i;
	dwt_readrxtimestamp(ts_tab);
//	TRACE("TS: ");
	for (i = 4; i >= 0; i--) {
		ts <<= 8;
		ts |= ts_tab[i];
//		TRACE("%02X", ts_tab[i]);
	}
//	TRACE("\r\n");
//	TRACE("TS2: %" PRIu64 "\r\n", ts);
	return ts;
}

double DWU_to_MS(uint64_t ts) {
	double temp = 0;
	temp = ts/(1000*499.2*128); //(MS_TO_DWT_TIME);
	return temp;
}

void cs_listener_run(void) {
	uint8_t ts[8];
	uint32 chirp_ts;
	int64_t rcv_tag_ts = 0;
	uint32 sync_rate = cph_config->sender_period;
	uint8 functionCode = 0;
	double diff_val = 0;
	double diff_val_prev = 0;
	double diff_val_var = 0;
	double diff_ts =0;
	double adjusted_ts = 0;
	double relative_tag_ts = 0;

	int64_t rcv_curr_ts = 0;
	int64_t rcv_prev_ts = 0;
	int64_t rcv_diff_ts = 0;
	double rcv_diff = 0;

	int64_t blink_curr_ts = 0;
	int64_t blink_prev_ts = 0;
	int64_t blink_diff_ts = 0;
	double blink_diff = 0;

	double rcv_blink_diff = 0;

	uint32 rcv_interval;
	uint32 elapsed;
	uint8 count = 1;

	irq_init();
	pio_disable_interrupt(DW_IRQ_PIO, DW_IRQ_MASK);
	cph_deca_init_device();
	cph_deca_init_network(cph_config->panid, cph_config->shortid);

	printf("Device ID: %08X\r\n", dwt_readdevid());


	// Enable external sync
	uint32_t ec_ctrl;
	dwt_readfromdevice(EXT_SYNC_ID, EC_CTRL_OFFSET, 4, (uint8_t*) &ec_ctrl);
	ec_ctrl &= EC_CTRL_WAIT_MASK;			// clear WAIT field
	ec_ctrl |= EC_CTRL_OSTRM | (33 << 3);	// turn on OSTRM and put 33 in WAIT field
	dwt_writetodevice(EXT_SYNC_ID, EC_CTRL_OFFSET, 4, (uint8_t*) &ec_ctrl);


	dwt_setcallbacks(0, rxcallback);

	chirp_ts = cph_get_millis();
	dwt_setinterrupt( DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO /*| DWT_INT_RXPTO*/), 1);
	pio_enable_interrupt(DW_IRQ_PIO, DW_IRQ_MASK);

	dwt_rxenable(0);

	while(1)
	{
		if(AWAKE_CHIRP) {
			elapsed = cph_get_millis() - chirp_ts;
			if(elapsed > CHIRP_PERIOD) {
				TRACE("Awake %d\r\n", count);
				count++;
				chirp_ts = cph_get_millis();
			}
		}

		if(irq_status == STATUS_RCV){
			functionCode = ((cph_deca_msg_header_t*)rx_buffer)->functionCode;

			switch(functionCode){
			case FUNC_CS_SYNC:
				dwt_readrxtimestamp(ts);
				TRACE("end   sys: ");
				for (int i = 4; i >= 0; i--) {
					TRACE("%02X", ts[i]);
				}
				TRACE("\r\n");
//				// Difference in RX timestamps
//				rcv_curr_ts = get_rx_timestamp();
//				rcv_diff_ts = rcv_curr_ts - rcv_prev_ts;
//				rcv_prev_ts = rcv_curr_ts;
//				rcv_diff = (int64_t)rcv_diff_ts * DWT_TIME_UNITS * 1000;
//
//				// Difference in TX timestamps
//				blink_curr_ts = ((int64_t)((cph_deca_msg_blink_t*)rx_buffer)->blinkTxTs) << 8;
//				blink_diff_ts = blink_curr_ts - blink_prev_ts;
//				blink_prev_ts = blink_curr_ts;
//				blink_diff = blink_diff_ts * DWT_TIME_UNITS * 1000;
//
//				rcv_blink_diff = rcv_diff - blink_diff;
//
////				TRACE("TS0: %08X \t TS1: %08X \t V: %08X \t %+.08f ms\r\n", blink_ts_prev, blink_ts, blink_ts_var, blink_diff);
//				TRACE("Bdiff: %+.08f ms \t Rdiff: %+.08f ms \t RBdiff: %+.08f ms\r\n", blink_diff, rcv_diff, rcv_blink_diff);

				break;
			case FUNC_CS_TAG:
				rcv_tag_ts = get_rx_timestamp();
				rcv_tag_ts = rcv_tag_ts - rcv_curr_ts;
				relative_tag_ts = DWU_to_MS(rcv_tag_ts);
//				TRACE("%02X : Raw Tag TS: %.08f   ", ((cph_deca_msg_header_t*)rx_buffer)->seq, relative_tag_ts);
				if(relative_tag_ts < 100) {
//					relative_tag_ts = ((relative_tag_ts - diff_ts) - (int)(relative_tag_ts - diff_ts)) * 1000000; //Truncates leading value, and converts to ns
					relative_tag_ts = ((relative_tag_ts - diff_ts) - (int)(relative_tag_ts - diff_ts)) * 1000; //Truncates leading value, and converts to μs
//					relative_tag_ts = (relative_tag_ts - diff_ts); //Account for offset from source clock in ms
					TRACE("%02X:%.08f\r\n", ((cph_deca_msg_header_t*)rx_buffer)->seq, relative_tag_ts);
				}
				break;
			case FUNC_CS_COORD:
				break;
			default:
				break;
			}
			irq_status = STATUS_EMPTY;
			dwt_rxenable(0);

		} else if (irq_status == STATUS_ERR) {
//			TRACE("INVALID LENGTH: %d\r\n", frame_len);
			irq_status = STATUS_EMPTY;
			dwt_rxenable(0);
		}
	}
}
