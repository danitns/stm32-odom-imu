/*
 * servo.h
 *
 *  Created on: Mar 1, 2025
 *      Author: danitns
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#define MIN_TURN_RADS -454  // Minimum turn position
#define MAX_TURN_RADS 384   // Maximum turn position
#define MIN_TURN_US 960
#define MID_TURN_US 1400
#define MAX_TURN_US 1840

void set_desired_position(int pos_rad, int* pos_us);
int get_position_in_rads(int pos_us);

#endif /* INC_SERVO_H_ */
