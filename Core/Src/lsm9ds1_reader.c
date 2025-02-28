/*
 * lsm9ds1_reader.c
 *
 *  Created on: Feb 27, 2025
 *      Author: danitns
 */

#include "lsm9ds1_reader.h"

void read_accel_gyro(stmdev_ctx_t *dev_ctx_imu, lsm9ds1_status_t reg,
		float_t *acceleration_mg, float_t *angular_rate_dps) {
	if (reg.status_imu.xlda && reg.status_imu.gda) {
		int16_t data_raw_acceleration[3];
		int16_t data_raw_angular_rate[3];

		memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
		memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
		lsm9ds1_acceleration_raw_get(dev_ctx_imu, data_raw_acceleration);
		lsm9ds1_angular_rate_raw_get(dev_ctx_imu, data_raw_angular_rate);
		acceleration_mg[0] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration[0])
				/ 1000.f * SENSORS_GRAVITY_EARTH;
		acceleration_mg[1] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration[1])
				/ 1000.f * SENSORS_GRAVITY_EARTH;
		acceleration_mg[2] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration[2])
				/ 1000.f * SENSORS_GRAVITY_EARTH;
		angular_rate_dps[0] = lsm9ds1_from_fs2000dps_to_mdps(
				data_raw_angular_rate[0]) / 1000.f;
		angular_rate_dps[1] = lsm9ds1_from_fs2000dps_to_mdps(
				data_raw_angular_rate[1]) / 1000.f;
		angular_rate_dps[2] = lsm9ds1_from_fs2000dps_to_mdps(
				data_raw_angular_rate[2]) / 1000.f;
	}
}

void read_magnetometer(stmdev_ctx_t *dev_ctx_mag, lsm9ds1_status_t reg,
		float_t *magnetic_field_mgauss) {
	if (reg.status_mag.zyxda) {
		int16_t data_raw_magnetic_field[3];
		/* Read magnetometer data */
		memset(data_raw_magnetic_field, 0x00, 3 * sizeof(int16_t));
		lsm9ds1_magnetic_raw_get(dev_ctx_mag, data_raw_magnetic_field);
		magnetic_field_mgauss[0] = lsm9ds1_from_fs16gauss_to_mG(
				data_raw_magnetic_field[0]) * 0.1f;
		magnetic_field_mgauss[1] = lsm9ds1_from_fs16gauss_to_mG(
				data_raw_magnetic_field[1]) * 0.1f;
		magnetic_field_mgauss[2] = lsm9ds1_from_fs16gauss_to_mG(
				data_raw_magnetic_field[2]) * 0.1f;
	}

}

/*
 * @brief  Write generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
int32_t platform_write_imu(void *handle, uint8_t reg,
		const uint8_t *bufp, uint16_t len) {
	sensbus_t *sensbus = (sensbus_t*) handle;
	HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
	I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
	return 0;
}

/*
 * @brief  Write generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
int32_t platform_write_mag(void *handle, uint8_t reg,
		const uint8_t *bufp, uint16_t len) {
	sensbus_t *sensbus = (sensbus_t*) handle;
	/* Write multiple command */
	reg |= 0x80;
	HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
	I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
	return 0;
}

/*
 * @brief  Read generic imu register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
int32_t platform_read_imu(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len) {
	sensbus_t *sensbus = (sensbus_t*) handle;
	HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
	I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}

/*
 * @brief  Read generic magnetometer register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
int32_t platform_read_mag(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len) {
	sensbus_t *sensbus = (sensbus_t*) handle;
	/* Read multiple command */
	reg |= 0x80;
	HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
	I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	return 0;
}
