/*
 * cph_stdio.h
 *
 *  Created on: Sep 23, 2015
 *      Author: ericrudisill
 */

#ifndef SRC_CPH_CPH_STDIO_H_
#define SRC_CPH_CPH_STDIO_H_

#include <cph.h>

void cph_stdio_init(void);
bool cph_stdio_dataready(void);
void cph_stdio_readc(uint8_t *data);


#endif /* SRC_CPH_CPH_STDIO_H_ */
