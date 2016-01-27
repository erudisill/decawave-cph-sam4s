#include <cph.h>
#include <deca_device_api.h>

cph_config_t * cph_config;



uint16_t cph_coordid = 0;

void configure_main(void);

static void init_config(void) {
	cph_config->magic[0] = 'C';
	cph_config->magic[1] = 'P';
	cph_config->magic[2] = 'H';
	cph_config->magic[3] = 'T';
	cph_config->fw_major = FW_MAJOR;
	cph_config->fw_minor = FW_MINOR;
	cph_config->hw_major = BOARD_REV_MAJOR;
	cph_config->hw_minor = BOARD_REV_MINOR;
	cph_config->panid = MAC_PAN_ID;
	cph_config->shortid = 0x1234;

	TRACE("HW:%2X.%02X  FW:%2X.%02X  PAN_ID:%04X  SHORT_ID:%04X\r\n",
			cph_config->hw_major,
			cph_config->hw_minor,
			cph_config->fw_major,
			cph_config->fw_minor,
			cph_config->panid,
			cph_config->shortid);
}

int main(void) {

#if defined(ANCHOR)
	g_cph_mode = CPH_MODE_ANCHOR;
#elif defined(TAG)
	g_cph_mode = CPH_MODE_TAG;
#elif defined(LISTENER)
	g_cph_mode = CPH_MODE_LISTENER;
#elif defined(SENDER)
	g_cph_mode = CPH_MODE_SENDER;
#endif

	sysclk_init();
	board_init();

	cph_millis_init();
	cph_stdio_init();

	/* Output demo infomation. */
	TRACE("\r\nCPH RTLS Version %2X.%02X\r\n", FW_MAJOR, FW_MINOR);

	uint32_t f = sysclk_get_cpu_hz();
	TRACE("CPU FREQ: %lu\r\n", f);

	// Blink LED for 5 seconds
	pio_set_pin_high(LED_STATUS1_IDX);
	for (int i = 0; i < (5 * 8); i++) {
		uint8_t c = 0x00;
		uart_read(CONSOLE_UART, &c);
		if (c == 'c') {
			configure_main();
			break;
		}
		pio_toggle_pin(LED_STATUS0_IDX);
		cph_millis_delay(125);
	}
	pio_set_pin_high(LED_STATUS0_IDX);

	init_config();

	cph_deca_spi_init();

	if (g_cph_mode == CPH_MODE_ANCHOR) {
		TRACE("Mode: ANCHOR\r\n");
		anchor_run();
	} else if (g_cph_mode == CPH_MODE_COORD) {
		TRACE("Mode: COORDINATOR\r\n");
		anchor_run();
	} else if (g_cph_mode == CPH_MODE_TAG) {
		TRACE("Mode: TAG\r\n");
		tag_run();
	} else if (g_cph_mode == CPH_MODE_LISTENER) {
		TRACE("Mode: LISTENER\r\n");
		listener_run();
	} else if (g_cph_mode == CPH_MODE_SENDER) {
		TRACE("Mode: SENDER\r\n");
		sender_run();
	}
}

