/*
 * imu.h
 *
 *  Created on: Oct 2, 2015
 *      Author: jcobb
 */

#ifndef IMU_H_
#define IMU_H_

#include "compiler.h"
#include "i2c.h"
#include "imu_def.h"
#include "imu_api.h"

extern volatile bool wake_event_received;

#define IMU_ADDRESS					0x68
#define IMU_BUFFER_LEN				14
#define IMU_INTERRUPT_ENABLE		1

extern uint8_t imu_buffer[I2C_BUFFER_LENGTH];


void imu_init(void);
void imu_init_wom(void);
void imu_init_lowpower_motion_detection(void);
void imu_init_gimbal(void);
void imu_run_console(void);






#endif /* IMU_H_ */
