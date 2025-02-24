/*
 * ahrs.h
 *
 *  Created on: Feb 18, 2025
 *      Author: danitns
 */

#ifndef INC_AHRS_H_
#define INC_AHRS_H_

#include <math.h>


#define sampleFreqDef 100.0f // sample frequency in Hz
#define beta 0.15f // 2 * proportional gain

float invSqrt(float x);
void updateIMU(float gx, float gy, float gz,
		float ax, float ay, float az,
		float dt, float *q0, float *q1, float *q2, float *q3);
void updateAHRS(float gx, float gy, float gz,
		float ax, float ay, float az,
		float mx, float my, float mz, float dt,
		float *q0, float *q1, float *q2, float *q3);
void calibrate_data(float_t *angular_data, float_t *acceleration_data, float_t *magnetic_data);


#endif /* INC_AHRS_H_ */
