#include <cph.h>
#include <deca_device_api.h>
#include <string.h>


cph_config_t * cph_config;
uint16_t cph_coordid = 0;

void configure_main(void);

//static void init_config(void) {
//	cph_config->magic[0] = 'C';
//	cph_config->magic[1] = 'P';
//	cph_config->magic[2] = 'H';
//	cph_config->magic[3] = 'T';
//	cph_config->fw_major = FW_MAJOR;
//	cph_config->fw_minor = FW_MINOR;
//	cph_config->hw_major = BOARD_REV_MAJOR;
//	cph_config->hw_minor = BOARD_REV_MINOR;
//	cph_config->panid = MAC_PAN_ID;
//	cph_config->shortid = 0x1234;
//
//	TRACE("HW:%2X.%02X  FW:%2X.%02X  PAN_ID:%04X  SHORT_ID:%04X\r\n",
//			cph_config->hw_major,
//			cph_config->hw_minor,
//			cph_config->fw_major,
//			cph_config->fw_minor,
//			cph_config->panid,
//			cph_config->shortid);
//}

static void init_config(void) {
	bool do_reset = false;

	cph_config = (cph_config_t*)cph_config_init();

	// If no magic, first run.
	if (cph_config->magic[0] != 'C' || cph_config->magic[1] != 'P' || cph_config->magic[2] != 'H' || cph_config->magic[3] != 'T') {
		do_reset = true;
	}
	// Not the first run, but if FW versions don't match, reset
	else if (cph_config->fw_major != FW_MAJOR || cph_config->fw_minor != FW_MINOR) {
		do_reset = true;
	}

	if (do_reset) {
		cph_config->magic[0] = 'C';
		cph_config->magic[1] = 'P';
		cph_config->magic[2] = 'H';
		cph_config->magic[3] = 'T';
		cph_config->fw_major = FW_MAJOR;
		cph_config->fw_minor = FW_MINOR;
		cph_config->hw_major = BOARD_REV_MAJOR;
		cph_config->hw_minor = BOARD_REV_MINOR;
		cph_config->panid = MAC_PAN_ID;

		cph_config->shortid = cph_utils_get_shortid_candidate();

#if defined(ANCHOR)
		cph_config->mode = CPH_MODE_ANCHOR;
#elif defined(TAG)
		cph_config->mode = CPH_MODE_TAG;
#elif defined(LISTENER)
		cph_config->mode = CPH_MODE_LISTENER;
#elif defined(SENDER)
		cph_config->mode = CPH_MODE_SENDER;
#endif
		memcpy(&cph_config->dwt_config, &g_dwt_configs[0], sizeof(dwt_config_t));
		cph_config->sender_period = POLL_DELAY_MS;
		cph_config_write();
	}

}

void print_greeting() {
	/* Output demo infomation. */
	TRACE("\r\nCPH RTLS Version %2X.%02X\r\n", FW_MAJOR, FW_MINOR);

	uint32_t f = sysclk_get_cpu_hz();
	TRACE("CPU FREQ: %lu\r\n", f);

	TRACE("HW:%2X.%02X  FW:%2X.%02X  PAN_ID:%04X  SHORT_ID:%04X\r\n",
			cph_config->hw_major,
			cph_config->hw_minor,
			cph_config->fw_major,
			cph_config->fw_minor,
			cph_config->panid,
			cph_config->shortid);

	if (cph_config->mode == CPH_MODE_ANCHOR) {
		TRACE("Mode: ANCHOR\r\n");
	} else if (cph_config->mode == CPH_MODE_COORD) {
		TRACE("Mode: COORDINATOR\r\n");
	} else if (cph_config->mode == CPH_MODE_TAG) {
		TRACE("Mode: TAG\r\n");
	} else if (cph_config->mode == CPH_MODE_LISTENER) {
		TRACE("Mode: LISTENER\r\n");
	} else if (cph_config->mode == CPH_MODE_SENDER) {
		TRACE("Mode: SENDER\r\n");
	} else {
		TRACE("Mode: UNKNOWN!\r\n");
	}

	TRACE("dwt_config: ");
	configure_print_dwt_config(&cph_config->dwt_config);
	TRACE("\r\n");
}

int main(void) {



	sysclk_init();
	board_init();

	cph_millis_init();
	cph_stdio_init();

	init_config();

	print_greeting();

	// Blink LED for 5 seconds
	pio_set_pin_high(LED_STATUS1_IDX);
	for (int i = 0; i < (5 * 8); i++) {
		uint8_t c = 0x00;
		uart_read(CONSOLE_UART, &c);
		if (c == 'c') {
			configure_main();
			init_config();
			TRACE("\r\nPOST CONFIG\r\n");
			print_greeting();
			break;
		}
		pio_toggle_pin(LED_STATUS0_IDX);
		cph_millis_delay(125);
	}
	pio_set_pin_high(LED_STATUS0_IDX);

	cph_deca_spi_init();

	if (cph_config->mode == CPH_MODE_ANCHOR) {
		anchor_run();
	} else if (cph_config->mode == CPH_MODE_COORD) {
		anchor_run();
	} else if (cph_config->mode == CPH_MODE_TAG) {
		tag_run();
	} else if (cph_config->mode == CPH_MODE_LISTENER) {
		listener_run();
	} else if (cph_config->mode == CPH_MODE_SENDER) {
		sender_run();
	}
}

