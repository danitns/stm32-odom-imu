/*
 * ahrs.c
 *
 *  Created on: Feb 18, 2025
 *      Author: danitns
 */
#include "ahrs.h"

//static const float_t mag_hardiron[3] = {26.79, 13.85, 10.00};
//static const float_t mag_softiron[9] =
//{
//	1.021, 0.024, -0.030,
//	0.024, 0.890, -0.066,
//	-0.030, -0.066, 1.106
//};
//static const float_t gyro_zerorate[3] = {0.05, -0.01, -0.01};
//static const float_t accel_zerog[3] = {0, 0, 0};

static const float_t mag_hardiron[3] = {27.36, 10.43, 4.24};
static const float_t mag_softiron[9] =
{
	1.111, 0.010, -0.031,
	0.010, 0.787, -0.118,
	-0.031, -0.118, 1.163
};
static const float_t gyro_zerorate[3] = {0.05, -0.01, -0.01};
static const float_t accel_zerog[3] = {0, 0, 0};

float invSqrt(float x) {
	float halfx = 0.5f * x;
	union {
		float f;
		long i;
	} conv = { x };
	conv.i = 0x5f3759df - (conv.i >> 1);
	conv.f *= 1.5f - (halfx * conv.f * conv.f);
	conv.f *= 1.5f - (halfx * conv.f * conv.f);
	return conv.f;
}

void updateIMU(float gx, float gy, float gz, float ax, float ay, float az,
		float dt, float *q0, float *q1, float *q2, float *q3) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1,
			q2q2, q3q3;

	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-(*q1) * gx - (*q2) * gy - (*q3) * gz);
	qDot2 = 0.5f * ((*q0) * gx + (*q2) * gz - (*q3) * gy);
	qDot3 = 0.5f * ((*q0) * gy - (*q1) * gz + (*q3) * gx);
	qDot4 = 0.5f * ((*q0) * gz + (*q1) * gy - (*q2) * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in
	// accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * (*q0);
		_2q1 = 2.0f * (*q1);
		_2q2 = 2.0f * (*q2);
		_2q3 = 2.0f * (*q3);
		_4q0 = 4.0f * (*q0);
		_4q1 = 4.0f * (*q1);
		_4q2 = 4.0f * (*q2);
		_8q1 = 8.0f * (*q1);
		_8q2 = 8.0f * (*q2);
		q0q0 = (*q0) * (*q0);
		q1q1 = (*q1) * (*q1);
		q2q2 = (*q2) * (*q2);
		q3q3 = (*q3) * (*q3);

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * (*q1) - _2q0 * ay - _4q1
				+ _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * (*q2) + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2
				+ _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * (*q3) - _2q1 * ax + 4.0f * q2q2 * (*q3) - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	(*q0) += qDot1 * dt;
	(*q1) += qDot2 * dt;
	(*q2) += qDot3 * dt;
	(*q3) += qDot4 * dt;

	// Normalise quaternion
	recipNorm = invSqrt(
			(*q0) * (*q0) + (*q1) * (*q1) + (*q2) * (*q2) + (*q3) * (*q3));
	(*q0) *= recipNorm;
	(*q1) *= recipNorm;
	(*q2) *= recipNorm;
	(*q3) *= recipNorm;
	//anglesComputed = 0;
}

void updateAHRS(float gx, float gy, float gz,
		float ax, float ay, float az,
		float mx, float my, float mz, float dt,
		float *q0, float *q1, float *q2, float *q3) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1,
			_2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2,
			q1q3, q2q2, q2q3, q3q3;

// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in
// magnetometer normalisation)
	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		updateIMU(gx, gy, gz, ax, ay, az, dt, q0, q1, q2, q3);
		return;
	}

// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-(*q1) * gx - (*q2) * gy - (*q3) * gz);
	qDot2 = 0.5f * ((*q0) * gx + (*q2) * gz - (*q3) * gy);
	qDot3 = 0.5f * ((*q0) * gy - (*q1) * gz + (*q3) * gx);
	qDot4 = 0.5f * ((*q0) * gz + (*q1) * gy - (*q2) * gx);

// Compute feedback only if accelerometer measurement valid (avoids NaN in
// accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * (*q0) * mx;
		_2q0my = 2.0f * (*q0) * my;
		_2q0mz = 2.0f * (*q0) * mz;
		_2q1mx = 2.0f * (*q1) * mx;
		_2q0 = 2.0f * (*q0);
		_2q1 = 2.0f * (*q1);
		_2q2 = 2.0f * (*q2);
		_2q3 = 2.0f * (*q3);
		_2q0q2 = 2.0f * (*q0) * (*q2);
		_2q2q3 = 2.0f * (*q2) * (*q3);
		q0q0 = (*q0) * (*q0);
		q0q1 = (*q0) * (*q1);
		q0q2 = (*q0) * (*q2);
		q0q3 = (*q0) * (*q3);
		q1q1 = (*q1) * (*q1);
		q1q2 = (*q1) * (*q2);
		q1q3 = (*q1) * (*q3);
		q2q2 = (*q2) * (*q2);
		q2q3 = (*q2) * (*q3);
		q3q3 = (*q3) * (*q3);

// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * (*q3) + _2q0mz * (*q2) + mx * q1q1 + _2q1 * my * (*q2)
				+ _2q1 * mz * (*q3) - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * (*q3) + my * q0q0 - _2q0mz * (*q1) + _2q1mx * (*q2) - my * q1q1
				+ my * q2q2 + _2q2 * mz * (*q3) - my * q3q3;
		_2bx = sqrtf(hx * hx + hy * hy);
		_2bz = -_2q0mx * (*q2) + _2q0my * (*q1) + mz * q0q0 + _2q1mx * (*q3) - mz * q1q1
				+ _2q2 * my * (*q3) - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax)
				+ _2q1 * (2.0f * q0q1 + _2q2q3 - ay)
				- _2bz * (*q2)
						* (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2)
								- mx)
				+ (-_2bx * (*q3) + _2bz * (*q1))
						* (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
				+ _2bx * (*q2)
						* (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2)
								- mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax)
				+ _2q0 * (2.0f * q0q1 + _2q2q3 - ay)
				- 4.0f * (*q1) * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
				+ _2bz * (*q3)
						* (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2)
								- mx)
				+ (_2bx * (*q2) + _2bz * (*q0))
						* (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
				+ (_2bx * (*q3) - _4bz * (*q1))
						* (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2)
								- mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax)
				+ _2q3 * (2.0f * q0q1 + _2q2q3 - ay)
				- 4.0f * (*q2) * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az)
				+ (-_4bx * (*q2) - _2bz * (*q0))
						* (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2)
								- mx)
				+ (_2bx * (*q1) + _2bz * (*q3))
						* (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
				+ (_2bx * (*q0) - _4bz * (*q2))
						* (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2)
								- mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax)
				+ _2q2 * (2.0f * q0q1 + _2q2q3 - ay)
				+ (-_4bx * (*q3) + _2bz * (*q1))
						* (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2)
								- mx)
				+ (-_2bx * (*q0) + _2bz * (*q2))
						* (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my)
				+ _2bx * (*q1)
						* (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2)
								- mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

// Integrate rate of change of quaternion to yield quaternion
	(*q0) += qDot1 * dt;
	(*q1) += qDot2 * dt;
	(*q2) += qDot3 * dt;
	(*q3) += qDot4 * dt;

// Normalise quaternion
	recipNorm = invSqrt((*q0) * (*q0) + (*q1) * (*q1) + (*q2) * (*q2) + (*q3) * (*q3));
	(*q0) *= recipNorm;
	(*q1) *= recipNorm;
	(*q2) *= recipNorm;
	(*q3) *= recipNorm;
	//anglesComputed = 0;
}

void calibrate_data(float_t *angular_data, float_t *acceleration_data, float_t *magnetic_data) {
	// hard iron cal
    float_t mxh = magnetic_data[0] - mag_hardiron[0];
    float_t myh = magnetic_data[1] - mag_hardiron[1];
    float_t mzh = magnetic_data[2] - mag_hardiron[2];
    // soft iron cal
    magnetic_data[0] = mxh * mag_softiron[0] + myh * mag_softiron[1] + mzh * mag_softiron[2];
    magnetic_data[0] *= (-1);
    magnetic_data[1] = mxh * mag_softiron[3] + myh * mag_softiron[4] + mzh * mag_softiron[5];
    magnetic_data[1] *= (-1);
    magnetic_data[2] = mxh * mag_softiron[6] + myh * mag_softiron[7] + mzh * mag_softiron[8];

    angular_data[0] -= gyro_zerorate[0];
    angular_data[1] -= gyro_zerorate[1];
    angular_data[1] *= (-1);
    angular_data[2] -= gyro_zerorate[2];

    acceleration_data[0] -= accel_zerog[0];
    acceleration_data[1] -= accel_zerog[1];
    acceleration_data[1] *= (-1);
    acceleration_data[2] -= accel_zerog[2];
}

//void computeAngles(float* roll, float* pitch, float* yaw, float **grav, float q0, float q1, float q2, float q3, bool* anglesComputed) {
//  (*roll) = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
//  (*pitch) = asinf(-2.0f * (q1 * q3 - q0 * q2));
//  (*yaw) = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);
//  (*grav)[0] = 2.0f * (q1 * q3 - q0 * q2);
//  (*grav)[1] = 2.0f * (q0 * q1 + q2 * q3);
//  (*grav)[2] = 2.0f * (q1 * q0 - 0.5f + q3 * q3);
//  (*anglesComputed) = 1;
//}
