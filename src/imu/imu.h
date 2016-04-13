/*
 * imu.h
 *
 *  Created on: Oct 2, 2015
 *      Author: jcobb
 */

#ifndef IMU_H_
#define IMU_H_

#include "compiler.h"
#include "imu_def.h"


#define IMU_ADDRESS					0x68
#define IMU_BUFFER_LEN				14
#define IMU_INTERRUPT_ENABLE		1


void imu_init_default(void);
void imu_init(void);
void imu_reset(void);
bool imu_test_connection(void);
void imu_set_clock_source(uint8_t source);
void imu_set_sleep_enabled(bool enabled);
uint8_t imu_get_device_id(void);
void imu_set_full_scale_gyro_range(uint8_t range);
void imu_set_full_scale_accel_range(uint8_t range);
void imu_getmotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
void imu_getmotion9(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz);
void imu_get_acceleration(int16_t *x, int16_t *y, int16_t *z);
void imu_get_rotation(int16_t *x, int16_t *y, int16_t *z);
void imu_get_mag(int16_t *x, int16_t *y, int16_t *z);


uint8_t imu_get_dlpf_mode();
void imu_set_dlpf_mode(uint8_t mode);
void imu_set_rate(uint8_t rate);
void imu_set_int_enabled(uint8_t enabled);
uint8_t imu_get_int_dataready_status(void);

bool imu_irq_ready(void);
void imu_irq_reset(void);



#endif /* IMU_H_ */
