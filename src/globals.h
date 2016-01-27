/*
 * globals.h
 *
 *  Created on: Jan 27, 2016
 *      Author: ericrudisill
 */

#ifndef SRC_GLOBALS_H_
#define SRC_GLOBALS_H_


#include <deca_device_api.h>

#define G_CONFIG_COUNT		3
#define G_CONFIG_USER_IDX	(G_CONFIG_COUNT - 1)

#define G_CONFIG_CURRENT_PTR	(&g_dwt_configs[g_config_idx])

extern dwt_config_t g_dwt_configs[];
extern int g_config_idx;

extern int g_cph_mode;



#endif /* SRC_GLOBALS_H_ */
