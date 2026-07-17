#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include <stdint.h>

typedef enum {
	SENSOR_ACCELEROMETER = 1 << 0,
	SENSOR_GYROSCOPE = 1 << 1,
	SENSOR_MAGNETOMETER = 1 << 2
} SensorComponent;

typedef struct {
	float acceleration[3];
	uint8_t valid;
} AccelerometerData;

typedef struct {
	float angular_rate[3];
	uint8_t valid;
} GyroscopeData;

typedef struct {
	float magnetic_field[3];
	uint8_t valid;
	uint8_t overflow;
} MagnetometerData;

typedef struct {
	float accelerometer_offset[3];
	float gyroscope_offset[3];
} ImuCalibrationData;

typedef struct {
	void *context;
	uint8_t initialized_components;
	uint8_t (*init)(void *context, uint8_t components);
	uint8_t (*calibrate)(void *context, ImuCalibrationData *calibration);
	uint8_t (*read_accelerometer)(void *context, AccelerometerData *data);
	uint8_t (*read_gyroscope)(void *context, GyroscopeData *data);
	uint8_t (*read_magnetometer)(void *context, MagnetometerData *data);
} ImuSensor;

uint8_t ImuSensor_Init(ImuSensor *sensor, uint8_t components);
uint8_t ImuSensor_Calibrate(ImuSensor *sensor, ImuCalibrationData *calibration);
uint8_t ImuSensor_ReadAccelerometer(ImuSensor *sensor, AccelerometerData *data);
uint8_t ImuSensor_ReadGyroscope(ImuSensor *sensor, GyroscopeData *data);
uint8_t ImuSensor_ReadMagnetometer(ImuSensor *sensor, MagnetometerData *data);

#endif
