/*
 * configure.c
 *
 *  Created on: Jan 27, 2016
 *      Author: ericrudisill
 */

#include <string.h>
#include <cph.h>
#include <globals.h>

#define BUFFER_SIZE 80
static char choice = 0;
static char buffer[BUFFER_SIZE];


#define PLEN_COUNT 8
static int plen_values[] = {
	DWT_PLEN_4096,
	DWT_PLEN_2048,
	DWT_PLEN_1536,
	DWT_PLEN_1024,
	DWT_PLEN_512,
	DWT_PLEN_256,
	DWT_PLEN_128,
	DWT_PLEN_64
};
static int plen_inputs[] = {
	4096,
	2048,
	1536,
	1024,
	512,
	256,
	128,
	64
};


#define PAC_COUNT 4
static int pac_values[] = {
		DWT_PAC8,
		DWT_PAC16,
		DWT_PAC32,
		DWT_PAC64
};
static int pac_inputs[] = {
	8,
	16,
	32,
	64
};

static void print_config(dwt_config_t * source) {
	int i;

	TRACE("chan:%02X  ", source->chan);

	if (source->prf == DWT_PRF_16M)
		TRACE("PRF_16M  ");
	else if (source->prf == DWT_PRF_64M) {
		TRACE("PRF_64M  ");
	}
	else {
		TRACE("PRF_??  ");
	}

	for (i=0;i<PLEN_COUNT;i++) {
		if (source->txPreambLength == plen_values[i]) {
			TRACE("PLEN_%-4d  ", plen_inputs[i]);
			break;
		}
	}
	if (i==PLEN_COUNT)
		TRACE("PLEN_??  ");

	for (i=0;i<PAC_COUNT;i++) {
		if (source->rxPAC == pac_values[i]) {
			TRACE("PAC_%-2d  ", pac_inputs[i]);
			break;
		}
	}
	if (i==PAC_COUNT)
		TRACE("PAC_??  ");

	TRACE("txCode:%02X  rxCode:%02X  nsSFD:%02X  ", source->txCode, source->rxCode, source->nsSFD);


	if (source->dataRate == DWT_BR_110K) {
		TRACE("BR_110K  ");
	}
	else if (source->dataRate == DWT_BR_850K) {
		TRACE("BR_850K  ");
	}
	else if (source->dataRate == DWT_BR_6M8) {
		TRACE("BR_6M8   ");
	}
	else {
		TRACE("BR_??  ");
	}

	if (source->phrMode == DWT_PHRMODE_STD) {
		TRACE("PHRMODE_STD  ");
	}
	else if (source->phrMode == DWT_PHRMODE_EXT) {
		TRACE("PHRMODE_EXT  ");
	}
	else {
		TRACE("PHRMODE_??");
	}

	TRACE("sfdTO:%d", source->sfdTO);
}

static bool parse_config_tuples(char * tuples, dwt_config_t * target) {
	int ch, prf, plen, pac, txcode, rxcode, nssfd, datarate, phrmode, sfdto;
	int j = 0;
	bool valid = true;

	if (sscanf(tuples, "%d %d %d %d %d %d %d %d %d %d", &ch, &prf, &plen, &pac, &txcode, &rxcode, &nssfd, &datarate, &phrmode, &sfdto) != 10) {
		TRACE("BAD INPUT\r\n");
		valid = false;
	}
	else {
		target->chan = ch;
		target->txCode = txcode;
		target->rxCode = rxcode;
		target->nsSFD = nssfd;
		target->sfdTO = sfdto;


		// Parse PRF
		//
		if (prf == 64) {
			target->prf = DWT_PRF_64M;
		}
		else if (prf == 16) {
			target->prf = DWT_PRF_16M;
		}
		else {
			TRACE("BAD PRF: 16 or 64\r\n");
			valid = false;
		}

		// Parse PLEN
		//
		for (j=0;j<PLEN_COUNT;j++) {
			if (plen == plen_inputs[j]) {
				target->txPreambLength = plen_values[j];
				break;
			}
		}
		if (j == PLEN_COUNT) {
			TRACE("BAD PLEN\r\n");
			valid = false;
		}


		// Parse PAC
		//
		for (j=0;j<PAC_COUNT;j++) {
			if (pac == pac_inputs[j]) {
				target->rxPAC = pac_values[j];
				break;
			}
		}
		if (j == PAC_COUNT) {
			TRACE("BAD PAC\r\n");
			valid = false;
		}


		// Parse Data Rate
		//
		if (datarate == 110) {
			target->dataRate = DWT_BR_110K;
		}
		else if (datarate == 850) {
			target->dataRate = DWT_BR_850K;
		}
		else if (datarate == 6) {
			target->dataRate = DWT_BR_6M8;
		}
		else {
			TRACE("BAD DATA RATE: 110, 850, or 6\r\n");
			valid = false;
		}


		// Parse PHRMODE
		if (phrmode == 0) {
			target->phrMode = DWT_PHRMODE_STD;
		}
		else if (phrmode == 1) {
			target->phrMode = DWT_PHRMODE_EXT;
		}
		else {
			TRACE("BAD PHRMODE: 0 for STD, 1 for EXT\r\n");
			valid = false;
		}
	}

	return valid;
}

static bool configure_user_defined(void) {

	int ch, prf, plen, pac, txcode, rxcode, nssfd, datarate, phrmode, sfdto;
	int j = 0;
	bool valid = false;

	while (!valid) {
		TRACE("\r\n\r\nUser Defined\r\n");
		TRACE("===============\r\n");
		TRACE("ENTER x to exit to Main Menu\r\n");
		TRACE("NOTE: Use 6 for 6M8, 0 for STD PHRMODE, 1 for EXT PHRMODE\r\n");
		TRACE("FORMAT: ch prf plen pac txcode rxcode nssfd datarate phrmode sfdto\r\n");
		TRACE("\r\n> ");

		int i = 0;
		memset(buffer, 0, BUFFER_SIZE);

		while ((choice = getchar() & 0xFF) != '\r') {
			TRACE("%c", choice);
			buffer[i++] = choice;
			if (i == BUFFER_SIZE) {
				TRACE("--OVERRUN--\r\n");
				break;
			}
		}

		TRACE("\r\n");

		if (buffer[0] == 'x') {
			TRACE("Exiting to Main Menu. No changes made.\r\n");
			return false;
		}

		valid = parse_config_tuples(buffer, &g_dwt_configs[G_CONFIG_USER_IDX]);
		if (valid)
			g_config_idx = G_CONFIG_USER_IDX;
	}

	return true;
}

void configure_main(void) {

	while (1) {
		TRACE("\r\n\r\nMain Menu\r\n");
		TRACE("==============\r\n");

		for (int i=0;i<G_CONFIG_COUNT - 1;i++) {
			TRACE("%d) ", i);
			print_config(&g_dwt_configs[i]);
			TRACE("\r\n");
		}

		TRACE("U) User defined\r\n");
		TRACE("A) Exit and run as ANCHOR\r\n");
		TRACE("C) Exit and run as COORDINATOR\r\n");
		TRACE("T) Exit and run as TAG\r\n");
		TRACE("L) Exit and run as LISTENER\r\n");
		TRACE("S) Exit and run as SENDER\r\n");

		TRACE("\r\nCurrent\r\n");
		TRACE("\r\n   ");
		print_config(&g_dwt_configs[g_config_idx]);
		TRACE("\r\n");

		TRACE("\r\n> ");

		choice = getchar() & 0xFF;
		TRACE("%c - ", choice);

		if (choice >= '0' && choice <= ('0' + G_CONFIG_COUNT - 1 - 1)) {
			TRACE("VALID\r\n");
			g_config_idx = choice - '0';
		}
		else if (choice == 'u' || choice == 'U') {
			configure_user_defined();
		}
		else if (choice == 'a' || choice == 'A') {
			g_cph_mode = CPH_MODE_ANCHOR;
			TRACE("Exiting as CPH_MODE_ANCHOR\r\n");
			break;
		}
		else if (choice == 'c' || choice == 'C') {
			g_cph_mode = CPH_MODE_COORD;
			TRACE("Exiting as CPH_MODE_COORD\r\n");
			break;
		}
		else if (choice == 't' || choice == 'T') {
			g_cph_mode = CPH_MODE_TAG;
			TRACE("Exiting as CPH_MODE_TAG\r\n");
			break;
		}
		else if (choice == 'l' || choice == 'L') {
			g_cph_mode = CPH_MODE_LISTENER;
			TRACE("Exiting as CPH_MODE_LISTENER\r\n");
			break;
		}
		else if (choice == 's' || choice == 'S') {
			g_cph_mode = CPH_MODE_SENDER;
			TRACE("Exiting as CPH_MODE_SENDER\r\n");
			break;
		}
		else {
			TRACE("NOT VALID\r\n");
		}
	};

	TRACE("\r\n");

	TRACE("Starting in ");
	for (int i=3;i>0;i--) {
		TRACE("%d ", i);
		cph_millis_delay(1000);
	}
}
