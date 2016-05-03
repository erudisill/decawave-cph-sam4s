/*
 * gimbal.c
 *
 *  Created on: Jan 6, 2016
 *      Author: John Cobb
 */

#include <stdint.h>
#include <string.h>
#include <math.h>


#include "cph_millis.h"
#include "imu.h"
#include "gimbal.h"
//#include "MadgwickAHRS.h"


float ax, ay, az, gx, gy, gz, mx, my, mz;
//float axis_gyro[3] = {0.0f, 0.0f, 0.0f};
//float axis_accel[3] = {0.0f, 0.0f, 0.0f};
//float axis_mag[3] = {0.0f, 0.0f, 0.0f};

int16_t a1, a2, a3, g1, g2, g3, m1, m2, m3;     // raw data arrays reading
//float ax, ay, az, gx, gy, gz, mx, my, mz; 		// variables to hold latest sensor data values
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    		// vector to hold quaternion
float yaw = 0.0f;
float pitch = 0.0f;
float roll = 0.0f;

pid_data_t pid_pitch;
pid_data_t pid_roll;
pid_data_t pid_yaw;

uint8_t mag_rate = 10;
uint32_t m_count = 0;
uint32_t t_now = 0;
uint32_t t_lastupdate = 0;
uint32_t t_lastprintf = 0;
float deltat = 0.0f;

void gimbal_calibrate_gyro(void);
void read_sensor_data_float(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz);
void read_sensor_data_int16(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz);
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);


gimbal_calibration_t gimbal_calibration = {false, 0, 0, 0, 0, 0, 0};

void gimbal_init(void)
{
	imu_init();

	// set rate of sensor
	imu_set_rate(7);
	cph_millis_delay(10);
	// set digitial lowpass filter
//	imu_set_dlpf_mode(MPU6050_DLPF_BW_20);
	imu_set_lpf(MPU6050_DLPF_BW_5);


	cph_millis_delay(10);

	TRACE("imu test: %d\r\n", imu_test_connection() == 1);
	cph_millis_delay(10);

	TRACE("imu running...\r\n");
}


void gimbal_tick(void)
{

#ifdef GYRO_CALIBRATION
	if(gimbal_calibration.calibrated == false) {
		TRACE("*** Gyro Calibration\r\n");
		TRACE("*** starting in 5 seconds.\r\n");
		TRACE("*** ensure placement on flat surface free from vibrations.\r\n");
		TRACE("\r\n");
		cph_millis_delay(5000);
		TRACE("calibrating gyro...\r\n");
		gimbal_calibrate_gyro();
	}
#endif

	if(imu_get_int_dataready_status() == 1) {

		m_count++;
//		imu_getmotion9(&a1, &a2, &a3, &g1, &g2, &g3, &m1, &m2, &m3);
//		imu_getmotion6(&a1, &a2, &a3, &g1, &g2, &g3);

		imu_get_acceleration(&a1, &a2, &a3);
		ax = a1*2.0f/32768.0f; // 2 g full range for accelerometer
		ay = a2*2.0f/32768.0f; // 2 g full range for accelerometer
		az = a3*2.0f/32768.0f; // 2 g full range for accelerometer

		imu_get_rotation(&g1, &g2, &g3);

#ifdef GYRO_CALIBRATION
		gx -= gimbal_calibration.gyro_offset_x;
		gy -= gimbal_calibration.gyro_offset_y;
		gz -= gimbal_calibration.gyro_offset_z;
#endif


		gx = g1*250.0f/32768.0f; // 250 deg/s full range for gyroscope
		gy = g2*250.0f/32768.0f; // 250 deg/s full range for gyroscope
		gz = g3*250.0f/32768.0f; // 250 deg/s full range for gyroscope

		//  The gyros and accelerometers can in principle be calibrated in addition to any factory calibration but they are generally
		//  pretty accurate. You can check the accelerometer by making sure the reading is +1 g in the positive direction for each axis.
		//  The gyro should read zero for each axis when the sensor is at rest. Small or zero adjustment should be needed for these sensors.
		//  The magnetometer is a different thing. Most magnetometers will be sensitive to circuit currents, computers, and
		//  other both man-made and natural sources of magnetic field. The rough way to calibrate the magnetometer is to record
		//  the maximum and minimum readings (generally achieved at the North magnetic direction). The average of the sum divided by two
		//  should provide a pretty good calibration offset. Don't forget that for the MPU9150, the magnetometer x- and y-axes are switched
		//  compared to the gyro and accelerometer!
		if(m_count > (1000/mag_rate)) {

			imu_get_mag(&m1, &m2, &m3);
			mx = m1*10.0f*1229.0f/4096.0f + 18.0f; // milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
			my = m2*10.0f*1229.0f/4096.0f + 70.0f; // apply calibration offsets in mG that correspond to your environment and magnetometer
			mz = m3*10.0f*1229.0f/4096.0f + 270.0f;

			m_count = 0;
		}
	} else {

		// otherwise escape the function no new data to process
		return;
	}

	t_now = cph_get_millis();
//	deltat = ((t_now - t_lastupdate)/1000000.0f); // set integration time by time elapsed since last filter update
	deltat = ((t_now - t_lastupdate)/1000.0f); // set integration time by time elapsed since last filter update
	t_lastupdate = t_now;

	// convert to radians
	gx = gx * (M_PI/180.0f);
	gy = gy * (M_PI/180.0f);
	gz = gz * (M_PI/180.0f);

	MadgwickQuaternionUpdate(ax, ay, az, gx, gy, gz,  mx,  my, mz);

	// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
	// In this coordinate system, the positive z-axis is down toward Earth.
	// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
	// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
	// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
	// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
	// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
	// applied in the correct order which for this configuration is yaw, pitch, and then roll.
	// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.

	yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
	pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
	roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
	pitch *= 180.0f / M_PI;
	yaw   *= 180.0f / M_PI - 3.0f; // Declination at Evansville, IN is 3 degrees W
	roll  *= 180.0f / M_PI;


	if((t_now - t_lastprintf) > 100) {
		t_lastprintf = t_now;

//		TRACE("ax: %.3f", ax);
//		TRACE(" ay: %.3f", ay);
//		TRACE(" az: %.3f", az);
//		TRACE(" mg\r\n");
//
//		TRACE("gx: %.3f", gx);
//		TRACE(" gy: %.3f", gy);
//		TRACE(" gz: %.3f", gz);
//		TRACE(" deg/s\r\n");
//
//		TRACE("mx: %.3f", mx);
//		TRACE(" my: %.3f", my);
//		TRACE(" mz: %.3f", mz);
//		TRACE(" mG\r\n");
//
//		TRACE("q0: %.5f", q[0]);
//		TRACE(" qx: %.5f", q[1]);
//		TRACE(" qy: %.5f", q[2]);
//		TRACE(" qx: %.5f", q[3]);
//		TRACE("\r\n");

//		TRACE("yaw, pitch, roll: ");
//		TRACE("%.2f, ", yaw);
//		TRACE("%.2f, ", pitch);
//		TRACE("%.2f, ", roll);
//		TRACE("\r\n");
//
//		TRACE("rate: %.2f", 1.0f/deltat);
//		TRACE(" Hz\r\n");

		TRACE("roll/pitch/yaw %f:%f:%f\r\n", roll, pitch, yaw);

	}
}



void gimbal_calibrate_gyro(void)
{
	int8_t tilt_detected = 0;
	int16_t calibration_counter = 0;

	int16_t gyro_x = 0;
	int16_t gyro_y = 0;
	int16_t gyro_z = 0;

	int16_t prev_gyro_x = 0;
	int16_t prev_gyro_y = 0;
	int16_t prev_gyro_z = 0;

	int16_t gyro_offset_x = 0;
	int16_t gyro_offset_y = 0;
	int16_t gyro_offset_z = 0;

	while(true) {

		cph_millis_delay(5);

		if (calibration_counter == GYRO_ITERATIONS) {
			TRACE("gyro calibration successful\r\n");
			break;

		}

		// make sure the data registers are ready
//		if(imu_get_int_dataready_status() == 0) {
//			cph_millis_delay(10);
//			continue;
//		}

		// check to see if this is our first pass
		// if so prep the variables
		if(calibration_counter == 0) {

			imu_get_rotation(&gyro_x, &gyro_y, &gyro_z);
			cph_millis_delay(10);
			gyro_offset_x = 0;
			gyro_offset_y = 0;
			gyro_offset_z = 0;

			prev_gyro_x = gyro_x;
			prev_gyro_y = gyro_y;
			prev_gyro_z = gyro_z;
		}

		imu_get_rotation(&gyro_x, &gyro_y, &gyro_z);

		// sanity tilt detection
		if(abs(prev_gyro_x - gyro_x) > TOL) {
			tilt_detected++;
			TRACE("gyro tilt detected x: %d\r\n", (prev_gyro_x - gyro_x));
		}

		// sanity tilt detection
		if(abs(prev_gyro_y - gyro_y) > TOL) {
			tilt_detected++;
			TRACE("gyro tilt detected y: %d\r\n", (prev_gyro_y - gyro_y));
		}

		// sanity tilt detection
		if(abs(prev_gyro_z - gyro_z) > TOL) {
			tilt_detected++;
			TRACE("gyro tilt detected z: %d\r\n", (prev_gyro_z - gyro_z));
		}

		gyro_offset_x += gyro_x/GYRO_ITERATIONS;
		gyro_offset_y += gyro_y/GYRO_ITERATIONS;
		gyro_offset_z += gyro_z/GYRO_ITERATIONS;

		calibration_counter++;

		prev_gyro_x = gyro_x;
		prev_gyro_y = gyro_y;
		prev_gyro_z = gyro_z;

		if(tilt_detected >= 1) {
			TRACE("gyro calibration failed, retrying...\r\n");
			cph_millis_delay(1000);
			calibration_counter = 0;
			tilt_detected = 0;
		}

	}
	TRACE("\r\n");
	TRACE("updating gyro calibration offsets...\r\n");

	gimbal_calibration.gyro_offset_x = gyro_offset_x;
	gimbal_calibration.gyro_offset_y = gyro_offset_y;
	gimbal_calibration.gyro_offset_z = gyro_offset_z;
	gimbal_calibration.calibrated = true;
}



//void read_sensor_data_int16(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz)
//{
//	// make sure the sensor is ready
//	if(imu_get_int_dataready_status() == 1) {
//
//		// mag counter
//		m_count++;
//
//		imu_get_acceleration(ax, ay, az);
//
//		// apply calibration offsets
//	//	ax -= config.acc_offset_x;
//	//	ay -= config.acc_offset_y;
//	//	az -= config.acc_offset_z;
//
//		ax = ax*2/32768;
//		ay = ay*2/32768;
//		az = az*2/32768;
//
//		imu_get_rotation(gx, gy, gz);
//
//		// apply calibration offsets
//	//	gx -= config.gyro_offset_x;
//	//	gy -= config.gyro_offset_y;
//	//	gz -= config.gyro_offset_z;
//
//
//		gx = gx*250/32768; // 250 deg/s full range for gyroscope
//		gy = gy*250/32768;
//		gz = gz*250/32768;
//
//		if (m_count > 1000/mag_rate) {
//			imu_get_mag(mx, my, mz);
//			mx = mx*10*1229/4096 + 18; // milliGauss (1229 microTesla per 2^12 bits, 10 mG per microTesla)
//			my = my*10*1229/4096 + 70; // apply calibration offsets in mG that correspond to your environment and magnetometer
//			mz = mz*10*1229/4096 + 270;
//			m_count = 0;
//		}
//	}
//
//}




// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;

	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;

	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;

	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

	// Integrate to yield quaternion
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

}
