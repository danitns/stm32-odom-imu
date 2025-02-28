/*
 * lsm9ds1_reader.h
 *
 *  Created on: Feb 27, 2025
 *      Author: danitns
 */

#ifndef INC_LSM9DS1_READER_H_
#define INC_LSM9DS1_READER_H_

#include "stm32l4xx_hal.h"
#include "lsm9ds1_reg.h"
#include <string.h>

typedef struct {
  void   *hbus;
  uint8_t i2c_address;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
} sensbus_t;

void read_accel_gyro(stmdev_ctx_t *dev_ctx_imu, lsm9ds1_status_t reg,
		float_t *acceleration_mg, float_t *angular_rate_dps);
void read_magnetometer(stmdev_ctx_t *dev_ctx_mag, lsm9ds1_status_t reg,
		float_t *magnetic_field_mgauss);
int32_t platform_write_imu(void *handle, uint8_t reg,
		const uint8_t *bufp, uint16_t len);
int32_t platform_read_imu(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);
int32_t platform_write_mag(void *handle, uint8_t reg,
		const uint8_t *bufp, uint16_t len);
int32_t platform_read_mag(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len);

#endif /* INC_LSM9DS1_READER_H_ */
