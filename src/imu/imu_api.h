/*
 * imu_api.h
 *
 *  Created on: Apr 22, 2016
 *      Author: jcobb
 */

#ifndef SRC_IMU_IMU_API_H_
#define SRC_IMU_IMU_API_H_

#include "compiler.h"
#include "imu_def.h"

void imu_run(void);
void imu_run_sleeptest(void);


void imu_init_default(void);



void imu_reset(void);
bool imu_test_connection(void);
uint8_t imu_get_int_enabled(void);
bool imu_get_cycle_bit(void);
bool imu_get_sleep_bit(void);
bool imu_get_standby_bit(void);
uint8_t imu_get_reg(uint8_t reg);
bool imu_get_reg_bit(uint8_t reg, uint8_t bit);
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


void imu_set_motion_detection_threshold(uint8_t threshold);
void imu_set_zero_motion_detection_threshold(uint8_t threshold);
void imu_set_motion_detection_duration(uint8_t threshold);
void imu_set_zero_motion_detection_duration(uint8_t threshold);
void imu_set_standby_x_accel_enabled(bool enabled);
void imu_set_standby_y_accel_enabled(bool enabled);
void imu_set_standby_z_accel_enabled(bool enabled);
void imu_set_standby_xyz_accel_enabled(bool enabled);
void imu_set_dhpf_mode(uint8_t bandwidth);

void imu_set_power_on_delay(uint8_t delay);
void imu_set_int_zero_motion_enabled(bool enabled);

void imu_wom_set_pwr_mgmt_1(void);
void imu_wom_set_pwr_mgmt_2(void);
void imu_wom_set_accel_lpf(void);
void imu_wom_enable_motion_interrupt(void);
void imu_wom_enable_accel_hardware_intel(void);
void imu_wom_set_wakeup_frequency(uint8_t frequency);
void imu_wom_enable_cycle_mode();

uint8_t imu_get_dlpf_mode();
void imu_set_dlpf_mode(uint8_t mode);
void imu_set_rate(uint8_t rate);
void imu_set_int_enabled(uint8_t enabled);
uint8_t imu_get_int_dataready_status(void);


uint8_t imu_get_accel_config2(void);
bool get_wake_cycle_enabled(void);
bool get_sleep_enabled(void);
bool get_standby_enabled(void);
bool get_temp_sensor_enabled(void);
uint8_t get_clock_source(void);
uint8_t get_wake_frequency(void);
bool get_standby_x_accel_enabled(void);
bool get_standby_y_accel_enabled(void);
bool get_standby_z_accel_enabled(void);
bool get_standby_x_gyro_enabled(void);
bool get_standby_y_gyro_enabled(void);
bool get_standby_z_gyro_enabled(void);
uint8_t get_motion_interrupt_enabled(void);
bool get_accel_intel_enabled(void);
bool get_accel_intel_mode(void);
uint8_t get_motion_threshold(void);

bool imu_irq_ready(void);
void imu_irq_reset(void);

#endif /* SRC_IMU_IMU_API_H_ */
