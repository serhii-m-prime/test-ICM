#include "imu_sensor.h"

uint8_t ImuSensor_Init(ImuSensor *sensor, uint8_t components) {
	if (sensor == 0 || sensor->init == 0) {
		return 0;
	}

	sensor->initialized_components = sensor->init(sensor->context, components);
	return sensor->initialized_components == components;
}

uint8_t ImuSensor_Calibrate(ImuSensor *sensor, ImuCalibrationData *calibration) {
	if (sensor == 0 || sensor->calibrate == 0) {
		return 0;
	}

	return sensor->calibrate(sensor->context, calibration);
}

uint8_t ImuSensor_ReadAccelerometer(ImuSensor *sensor, AccelerometerData *data) {
	if (sensor == 0 || data == 0 || sensor->read_accelerometer == 0
			|| !(sensor->initialized_components & SENSOR_ACCELEROMETER)) {
		return 0;
	}

	return sensor->read_accelerometer(sensor->context, data);
}

uint8_t ImuSensor_ReadGyroscope(ImuSensor *sensor, GyroscopeData *data) {
	if (sensor == 0 || data == 0 || sensor->read_gyroscope == 0
			|| !(sensor->initialized_components & SENSOR_GYROSCOPE)) {
		return 0;
	}

	return sensor->read_gyroscope(sensor->context, data);
}

uint8_t ImuSensor_ReadMagnetometer(ImuSensor *sensor, MagnetometerData *data) {
	if (sensor == 0 || data == 0 || sensor->read_magnetometer == 0
			|| !(sensor->initialized_components & SENSOR_MAGNETOMETER)) {
		return 0;
	}

	return sensor->read_magnetometer(sensor->context, data);
}
