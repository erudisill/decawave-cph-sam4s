#include <cph.h>
#include <deca_device_api.h>

cph_config_t * cph_config;

#ifdef ANCHOR
int cph_mode = CPH_MODE_ANCHOR;
#else
int cph_mode = CPH_MODE_TAG;
#endif

uint16_t cph_coordid = 0;

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

#if 1

int main(void) {
	sysclk_init();
	board_init();

	cph_millis_init();
	cph_stdio_init();

	/* Output demo infomation. */
	TRACE(APP_NAME, FW_MAJOR, FW_MINOR);

	uint32_t f = sysclk_get_cpu_hz();
	TRACE("CPU FREQ: %lu\r\n", f);

	// Blink LED for 5 seconds
	pio_set_pin_high(LED_STATUS1_IDX);
	for (int i = 0; i < (5 * 8); i++) {
		pio_toggle_pin(LED_STATUS0_IDX);
		cph_millis_delay(125);
	}
	pio_set_pin_high(LED_STATUS0_IDX);

	init_config();

	cph_deca_spi_init();

	app_run();

	while (1) {
		uint32_t id = dwt_readdevid();
		uint32_t id2 = dwt_readdevid();
		uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);
		TRACE("id:%08X %08X   status:%08X\r\n", id, id2, status);
		cph_millis_delay(500);
	}
}

#endif

#if 0

int main(void) {
	sysclk_init();
	board_init();

	cph_stdio_init();
	cph_millis_init();

	/* Output demo infomation. */
	printf("\r\n-- Decawave SPI TEST --\n\r");

	// Blink LED for 2 seconds
	pio_set_pin_high(LED_STATUS1_IDX);
	for (int i = 0; i < (2 * 8); i++) {
		pio_toggle_pin(LED_STATUS0_IDX);
		cph_millis_delay(125);
	}
	pio_set_pin_high(LED_STATUS0_IDX);

	spi_set_master_mode(DW_SPI);

	pio_configure_pin(PIO_PA12_IDX, PIO_PERIPH_A);	// MISO
	pio_configure_pin(PIO_PA13_IDX, PIO_PERIPH_A);	// MOSI
	pio_configure_pin(PIO_PA14_IDX, PIO_PERIPH_A);	// SPCK
	pio_configure_pin(DW_CSn_PIO_IDX, DW_CSn_PIO_PERIPH);
	pmc_enable_periph_clk(ID_SPI);

	spi_disable(DW_SPI);
	spi_set_clock_polarity(DW_SPI, DW_CHIP_SELECT, DW_CLOCK_POLARITY);
	spi_set_clock_phase(DW_SPI, DW_CHIP_SELECT, DW_CLOCK_PHASE);
//	spi_set_baudrate_div(DW_SPI, DW_CHIP_SELECT, (sysclk_get_cpu_hz() / DW_SPI_BAUD_SLOW));
	spi_set_baudrate_div(DW_SPI, DW_CHIP_SELECT, (sysclk_get_cpu_hz() / 20000000));
//	spi_set_transfer_delay(DW_SPI, DW_CHIP_SELECT, DW_DELAY_BEFORE,		DW_DELAY_BETWEEN);
	spi_set_transfer_delay(DW_SPI, DW_CHIP_SELECT, 0x05,		0x05);
	spi_set_delay_between_chip_select(DW_SPI, 0x05);

	spi_set_fixed_peripheral_select(DW_SPI);
	spi_configure_cs_behavior(DW_SPI, DW_CHIP_SELECT, SPI_CS_KEEP_LOW);
	spi_set_peripheral_chip_select_value(DW_SPI, (~(1U << DW_CHIP_SELECT)));

	spi_enable(DW_SPI);

	uint8_t pcs = DW_CHIP_SELECT;
	uint16_t data = 0;
	uint8_t buff[5];


	spi_write(DW_SPI, 0, 0, 0);
	spi_read(DW_SPI, &data,&pcs);
	buff[0] = data & 0xFF;

	spi_write(DW_SPI, 0xFF, 0, 0);
	spi_read(DW_SPI, &data,&pcs);
	buff[1] = data & 0xFF;

	spi_write(DW_SPI, 0xFF, 0, 0);
	spi_read(DW_SPI, &data,&pcs);
	buff[2] = data & 0xFF;

	spi_write(DW_SPI, 0xFF, 0, 0);
	spi_read(DW_SPI, &data,&pcs);
	buff[3] = data & 0xFF;

	spi_set_lastxfer(DW_SPI);
	spi_write(DW_SPI, 0xFF, 0, 0);
	spi_read(DW_SPI, &data,&pcs);
	buff[4] = data & 0xFF;

//	for (int i=0;i<128;i++) {
//		asm("nop");
//	}

	while (spi_is_tx_empty(DW_SPI) == 0) ;

//	printf("returned %02X%02X%02X%02X\r\n", buff[1], buff[2], buff[3], buff[4]);


	spi_write(DW_SPI, 0, 0, 0);
	spi_read(DW_SPI, &data,&pcs);
	buff[0] = data & 0xFF;

	spi_write(DW_SPI, 0xFF, 0, 0);
	spi_read(DW_SPI, &data,&pcs);
	buff[1] = data & 0xFF;

	spi_write(DW_SPI, 0xFF, 0, 0);
	spi_read(DW_SPI, &data,&pcs);
	buff[2] = data & 0xFF;

	spi_write(DW_SPI, 0xFF, 0, 0);
	spi_read(DW_SPI, &data,&pcs);
	buff[3] = data & 0xFF;

	spi_set_lastxfer(DW_SPI);
	spi_write(DW_SPI, 0xFF, 0, 0);
	spi_read(DW_SPI, &data,&pcs);
	buff[4] = data & 0xFF;



	printf("returned %02X%02X%02X%02X\r\n", buff[1], buff[2], buff[3], buff[4]);

	while (1) ;

}
#endif
