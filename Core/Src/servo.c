/*
 * servo.c
 *
 *  Created on: Mar 1, 2025
 *      Author: danitns
 */
#include "servo.h"


long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void set_desired_position(int pos_rad, int* pos_us) {
	if(pos_rad < MIN_TURN_RADS) {
		pos_rad = MIN_TURN_RADS;
	} else if(pos_rad > MAX_TURN_RADS) {
		pos_rad = MAX_TURN_RADS;
	}

	if(pos_rad < 0) {
		(*pos_us) = map(pos_rad, MIN_TURN_RADS, 0, MAX_TURN_US, MID_TURN_US);
	} else {
		(*pos_us) = map(pos_rad, 0, MAX_TURN_RADS, MID_TURN_US, MIN_TURN_US);
	}
}

int get_position_in_rads(int pos_us) {
	if(pos_us < MID_TURN_US) {
		return map(pos_us, MIN_TURN_US, MID_TURN_US, MAX_TURN_RADS, 0);
	}
	return map(pos_us, MID_TURN_US, MAX_TURN_US, 0, MIN_TURN_RADS);
}
