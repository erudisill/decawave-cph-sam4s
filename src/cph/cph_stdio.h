/*
 * cph_stdio.h
 *
 *  Created on: Sep 23, 2015
 *      Author: ericrudisill
 */

#ifndef SRC_CPH_CPH_STDIO_H_
#define SRC_CPH_CPH_STDIO_H_

#include <cph.h>

typedef void (*cph_stdio_dataready_cb_t)(void);

void cph_stdio_init(void);
bool cph_stdio_dataready(void);
void cph_stdio_readc(uint8_t *data);
void cph_stdio_set_datready_cb(cph_stdio_dataready_cb_t dataready_cb);
int cph_stdio_read_buf(uint8_t *data, int maxlen);

#endif /* SRC_CPH_CPH_STDIO_H_ */
