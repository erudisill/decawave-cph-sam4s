/*
 * baro.h
 *
 *  Created on: Apr 14, 2016
 *      Author: jcobb
 */

#ifndef SRC_BARO_BARO_H_
#define SRC_BARO_BARO_H_

#include "compiler.h"
#include "baro_def.h"

void baro_init(void);
bool baro_begin(void);
uint32_t baro_get_pressure(void);
uint32_t baro_get_altitude(void);
uint32_t baro_get_temperature(void);

void write8(uint8_t a, uint8_t d);
uint8_t read8(uint8_t a);
uint8_t baro_mode;

#endif /* SRC_BARO_BARO_H_ */
