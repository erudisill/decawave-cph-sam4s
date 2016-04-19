/*
 * gimbal.h
 *
 *  Created on: Jan 6, 2016
 *      Author: jcobb
 */

#ifndef SRC_GIMBAL_GIMBAL_H_
#define SRC_GIMBAL_GIMBAL_H_


#define	GYRO_CALIBRATION			1
#define TOL							64
#define GYRO_ITERATIONS				4000

enum axis_def {
	X = 0,
	Y,
	Z
};

typedef struct pid_data {
  int32_t   kp, ki, kd;
} pid_data_t;

extern pid_data_t pid_pitch;
extern pid_data_t pid_roll;
extern pid_data_t pid_yaw;

typedef struct{
	bool calibrated;
	int16_t gyro_offset_x;
	int16_t gyro_offset_y;
	int16_t gyro_offset_z;
	int16_t accel_offset_x;
	int16_t accel_offset_y;
	int16_t accel_offset_z;
} gimbal_calibration_t;

extern gimbal_calibration_t gimbal_calibration;

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
#define GyroMeasError M_PI * (40.0f / 180.0f)       // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift M_PI * (0.0f / 180.0f)      // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f


extern float yaw;
extern float pitch;
extern float roll;


void gimbal_init(void);
void gimbal_tick(void);


void gimbal_get_orientation(void);


#endif /* SRC_GIMBAL_GIMBAL_H_ */
