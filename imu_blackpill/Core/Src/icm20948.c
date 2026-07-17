#include "icm20948.h"

#define REG_B0_WHO_AM_I 0x00
#define REG_B0_USER_CTRL 0x03
#define REG_BANK_SEL 0x7F
#define BANK0 0
#define REG_B0_PWR_MGMT_1 0x06
#define REG_B0_PWR_MGMT_1_DEVICE_RESET 0x80
#define REG_B0_PWR_MGMT_1_CLKSEL_2_0 0x01
#define REG_B0_PWR_MGMT_2 0x07
#define REG_B0_ACCEL_XOUT_H 0x2D
#define REG_B0_GYRO_XOUT_H 0x33
#define REG_B0_EXT_SLV_SENS_DATA_00 0x3B
#define BANK2 2
#define REG_B2_GYRO_CONFIG_1 0x01
#define REG_B2_ACCEL_CONFIG 0x14

#define BANK3 3
#define REG_B3_I2C_MST_CTRL 0x01
#define REG_B3_I2C_SLV0_ADDR 0x03
#define REG_B3_I2C_SLV0_REG 0x04
#define REG_B3_I2C_SLV0_CTRL 0x05
#define REG_B3_I2C_SLV0_DO 0x06

#define USER_CTRL_I2C_MST_EN 0x20
#define USER_CTRL_I2C_IF_DIS 0x10
#define I2C_SLV_READ 0x80
#define I2C_SLV_ENABLE 0x80
#define AK09916_I2C_ADDRESS 0x0C
#define AK09916_REG_WIA2 0x01
#define AK09916_REG_ST1 0x10
#define AK09916_REG_CNTL2 0x31
#define AK09916_REG_CNTL3 0x32
#define AK09916_DEVICE_ID 0x09
#define AK09916_DATA_LENGTH 9
#define AK09916_SCALE_UT 0.15f

static void ICM20948_Select(ICM20948 *imu, GPIO_PinState state) {
	HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, state);
}

static void ICM20948_SelectBank(ICM20948 *imu, uint8_t bank) {
	uint8_t tx_buf[2] = { REG_BANK_SEL, (bank << 4) & 0x30 };

	ICM20948_Select(imu, GPIO_PIN_RESET);
	HAL_SPI_Transmit(imu->spi, tx_buf, 2, HAL_MAX_DELAY);
	ICM20948_Select(imu, GPIO_PIN_SET);
}

static void ICM20948_WriteRegister(ICM20948 *imu, uint8_t bank, uint8_t reg, uint8_t value) {
	uint8_t tx_buf[2] = { reg & 0x7F, value };

	ICM20948_SelectBank(imu, bank);
	ICM20948_Select(imu, GPIO_PIN_RESET);
	HAL_SPI_Transmit(imu->spi, tx_buf, 2, HAL_MAX_DELAY);
	ICM20948_Select(imu, GPIO_PIN_SET);
}

static uint8_t ICM20948_ReadRegister(ICM20948 *imu, uint8_t bank, uint8_t reg) {
	uint8_t tx_buf[2] = { reg | 0x80, 0x00 };
	uint8_t rx_buf[2] = { 0, 0 };

	ICM20948_SelectBank(imu, bank);
	ICM20948_Select(imu, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(imu->spi, tx_buf, rx_buf, 2, HAL_MAX_DELAY);
	ICM20948_Select(imu, GPIO_PIN_SET);

	return rx_buf[1];
}

static void ICM20948_ReadBytes(ICM20948 *imu, uint8_t bank, uint8_t reg,
		uint8_t *data, uint16_t size) {
	uint8_t tx_buf = reg | 0x80;

	ICM20948_SelectBank(imu, bank);
	ICM20948_Select(imu, GPIO_PIN_RESET);
	HAL_SPI_Transmit(imu->spi, &tx_buf, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(imu->spi, data, size, HAL_MAX_DELAY);
	ICM20948_Select(imu, GPIO_PIN_SET);
}

static void ICM20948_MagnetometerWrite(ICM20948 *imu, uint8_t reg,
		uint8_t value) {
	ICM20948_WriteRegister(imu, BANK3, REG_B3_I2C_SLV0_ADDR,
			AK09916_I2C_ADDRESS);
	ICM20948_WriteRegister(imu, BANK3, REG_B3_I2C_SLV0_REG, reg);
	ICM20948_WriteRegister(imu, BANK3, REG_B3_I2C_SLV0_DO, value);
	ICM20948_WriteRegister(imu, BANK3, REG_B3_I2C_SLV0_CTRL,
			I2C_SLV_ENABLE | 1);
	HAL_Delay(10);
	ICM20948_WriteRegister(imu, BANK3, REG_B3_I2C_SLV0_CTRL, 0);
}

static uint8_t ICM20948_MagnetometerReadRegister(ICM20948 *imu,
		uint8_t reg) {
	uint8_t value;

	ICM20948_WriteRegister(imu, BANK3, REG_B3_I2C_SLV0_ADDR,
			I2C_SLV_READ | AK09916_I2C_ADDRESS);
	ICM20948_WriteRegister(imu, BANK3, REG_B3_I2C_SLV0_REG, reg);
	ICM20948_WriteRegister(imu, BANK3, REG_B3_I2C_SLV0_CTRL,
			I2C_SLV_ENABLE | 1);
	HAL_Delay(10);
	value = ICM20948_ReadRegister(imu, BANK0, REG_B0_EXT_SLV_SENS_DATA_00);
	ICM20948_WriteRegister(imu, BANK3, REG_B3_I2C_SLV0_CTRL, 0);

	return value;
}

static uint8_t ICM20948_InitMagnetometer(ICM20948 *imu) {
	uint8_t user_ctrl = ICM20948_ReadRegister(imu, BANK0, REG_B0_USER_CTRL);

	ICM20948_WriteRegister(imu, BANK0, REG_B0_USER_CTRL,
			user_ctrl | USER_CTRL_I2C_MST_EN | USER_CTRL_I2C_IF_DIS);
	ICM20948_WriteRegister(imu, BANK3, REG_B3_I2C_MST_CTRL, 0x07);
	ICM20948_MagnetometerWrite(imu, AK09916_REG_CNTL3, 0x01);
	HAL_Delay(100);

	if (ICM20948_MagnetometerReadRegister(imu, AK09916_REG_WIA2)
			!= AK09916_DEVICE_ID) {
		return 0;
	}

	ICM20948_MagnetometerWrite(imu, AK09916_REG_CNTL2, IMU_MAG_CONFIG);
	HAL_Delay(10);
	ICM20948_WriteRegister(imu, BANK3, REG_B3_I2C_SLV0_ADDR,
			I2C_SLV_READ | AK09916_I2C_ADDRESS);
	ICM20948_WriteRegister(imu, BANK3, REG_B3_I2C_SLV0_REG,
			AK09916_REG_ST1);
	ICM20948_WriteRegister(imu, BANK3, REG_B3_I2C_SLV0_CTRL,
			I2C_SLV_ENABLE | AK09916_DATA_LENGTH);

	return 1;
}

void ICM20948_Create(ICM20948 *imu, SPI_HandleTypeDef *spi,
		GPIO_TypeDef *cs_port, uint16_t cs_pin) {
	imu->spi = spi;
	imu->cs_port = cs_port;
	imu->cs_pin = cs_pin;
	imu->initialized_components = 0;

	for (uint8_t i = 0; i < 3; i++) {
		imu->accel_offset[i] = 0.0f;
		imu->gyro_offset[i] = 0.0f;
	}
}

static uint8_t ICM20948_InitComponents(void *context, uint8_t components) {
	ICM20948 *imu = context;
	uint8_t power_management_2 = 0;
	imu->initialized_components = 0;

	if (ICM20948_ReadRegister(imu, BANK0, REG_B0_WHO_AM_I) != 0xEA) {
		return 0;
	}

	ICM20948_WriteRegister(imu, BANK0, REG_B0_PWR_MGMT_1, REG_B0_PWR_MGMT_1_DEVICE_RESET);
	HAL_Delay(100);
	ICM20948_WriteRegister(imu, BANK0, REG_B0_PWR_MGMT_1, REG_B0_PWR_MGMT_1_CLKSEL_2_0);
	HAL_Delay(10);

	if (!(components & SENSOR_GYROSCOPE)) {
		power_management_2 |= 0x07;
	}
	if (!(components & SENSOR_ACCELEROMETER)) {
		power_management_2 |= 0x38;
	}
	ICM20948_WriteRegister(imu, BANK0, REG_B0_PWR_MGMT_2, power_management_2);
	HAL_Delay(10);

	if (components & SENSOR_GYROSCOPE) {
		ICM20948_WriteRegister(imu, BANK2, REG_B2_GYRO_CONFIG_1,
				IMU_GYRO_CONFIG);
		imu->initialized_components |= SENSOR_GYROSCOPE;
	}
	if (components & SENSOR_ACCELEROMETER) {
		ICM20948_WriteRegister(imu, BANK2, REG_B2_ACCEL_CONFIG,
				IMU_ACCEL_CONFIG);
		imu->initialized_components |= SENSOR_ACCELEROMETER;
	}
	if ((components & SENSOR_MAGNETOMETER)
			&& ICM20948_InitMagnetometer(imu)) {
		imu->initialized_components |= SENSOR_MAGNETOMETER;
	}

	return imu->initialized_components;
}

static uint8_t ICM20948_ReadAccelerometer(void *context,
		AccelerometerData *data) {
	ICM20948 *imu = context;
	uint8_t raw_data[6];

	ICM20948_ReadBytes(imu, BANK0, REG_B0_ACCEL_XOUT_H, raw_data, 6);
	for (uint8_t i = 0; i < 3; i++) {
		int16_t raw_value = (int16_t) ((raw_data[i * 2] << 8)
				| raw_data[i * 2 + 1]);
		data->acceleration[i] = ((float) raw_value / 8192.0f)
				- imu->accel_offset[i];
	}
	data->valid = 1;

	return 1;
}

static uint8_t ICM20948_ReadGyroscope(void *context, GyroscopeData *data) {
	ICM20948 *imu = context;
	uint8_t raw_data[6];

	ICM20948_ReadBytes(imu, BANK0, REG_B0_GYRO_XOUT_H, raw_data, 6);
	for (uint8_t i = 0; i < 3; i++) {
		int16_t raw_value = (int16_t) ((raw_data[i * 2] << 8)
				| raw_data[i * 2 + 1]);
		data->angular_rate[i] = ((float) raw_value / 65.5f)
				- imu->gyro_offset[i];
	}
	data->valid = 1;

	return 1;
}

static uint8_t ICM20948_ReadMagnetometer(void *context,
		MagnetometerData *data) {
	ICM20948 *imu = context;
	uint8_t raw_data[AK09916_DATA_LENGTH];
	uint8_t data_ready;

	ICM20948_ReadBytes(imu, BANK0, REG_B0_EXT_SLV_SENS_DATA_00,
			raw_data, AK09916_DATA_LENGTH);
	data_ready = raw_data[0] & 0x01;
	data->overflow = (raw_data[8] >> 3) & 0x01;

	if (data_ready && !data->overflow) {
		for (uint8_t i = 0; i < 3; i++) {
			int16_t raw_value = (int16_t) ((raw_data[2 + i * 2] << 8)
					| raw_data[1 + i * 2]);
			data->magnetic_field[i] = (float) raw_value * AK09916_SCALE_UT;
		}
		data->valid = 1;
	}
	if (data->overflow) {
		data->valid = 0;
	}

	return data_ready && !data->overflow;
}

static uint8_t ICM20948_CalibrateComponents(void *context,
		ImuCalibrationData *calibration) {
	ICM20948 *imu = context;
	AccelerometerData accelerometer;
	GyroscopeData gyroscope;
	float accel_sum[3] = { 0.0f, 0.0f, 0.0f };
	float gyro_sum[3] = { 0.0f, 0.0f, 0.0f };
	const uint16_t samples = 500;

	for (uint8_t axis = 0; axis < 3; axis++) {
		imu->accel_offset[axis] = 0.0f;
		imu->gyro_offset[axis] = 0.0f;
	}

	for (uint16_t sample = 0; sample < samples; sample++) {
		if (imu->initialized_components & SENSOR_ACCELEROMETER) {
			ICM20948_ReadAccelerometer(imu, &accelerometer);
		}
		if (imu->initialized_components & SENSOR_GYROSCOPE) {
			ICM20948_ReadGyroscope(imu, &gyroscope);
		}
		for (uint8_t axis = 0; axis < 3; axis++) {
			if (imu->initialized_components & SENSOR_ACCELEROMETER) {
				accel_sum[axis] += accelerometer.acceleration[axis];
			}
			if (imu->initialized_components & SENSOR_GYROSCOPE) {
				gyro_sum[axis] += gyroscope.angular_rate[axis];
			}
		}
		HAL_Delay(5);
	}

	for (uint8_t axis = 0; axis < 3; axis++) {
		if (imu->initialized_components & SENSOR_ACCELEROMETER) {
			imu->accel_offset[axis] = accel_sum[axis] / (float) samples;
		}
		if (imu->initialized_components & SENSOR_GYROSCOPE) {
			imu->gyro_offset[axis] = gyro_sum[axis] / (float) samples;
		}
	}
	if (imu->initialized_components & SENSOR_ACCELEROMETER) {
		imu->accel_offset[2] -= 1.0f;
	}

	for (uint8_t axis = 0; axis < 3; axis++) {
		calibration->accelerometer_offset[axis] = imu->accel_offset[axis];
		calibration->gyroscope_offset[axis] = imu->gyro_offset[axis];
	}

	return 1;
}

void ICM20948_Bind(ImuSensor *sensor, ICM20948 *imu) {
	sensor->context = imu;
	sensor->initialized_components = 0;
	sensor->init = ICM20948_InitComponents;
	sensor->calibrate = ICM20948_CalibrateComponents;
	sensor->read_accelerometer = ICM20948_ReadAccelerometer;
	sensor->read_gyroscope = ICM20948_ReadGyroscope;
	sensor->read_magnetometer = ICM20948_ReadMagnetometer;
}
