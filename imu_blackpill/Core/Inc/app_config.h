#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdbool.h>
#include "imu_sensor.h"

/* Only the accelerometer is initialized in the current firmware build. */
#define APP_REQUIRED_SENSOR_COMPONENTS SENSOR_ACCELEROMETER

/* UART stream selection for the filter comparison page. */
#define APP_UART_LEGACY_IMU_STREAM_ENABLED       0
#define APP_UART_ACCEL_COMPARISON_STREAM_ENABLED 1
#define APP_UART_FFT_STREAM_ENABLED              1
#define APP_UART_STATUS_STREAM_ENABLED           1

/* The comparison stream and FFT include all accelerometer axes: X, Y, Z. */
#define APP_ACCEL_AXIS_COUNT     3U
#define APP_ACCEL_ALL_AXES_MASK ((1U << APP_ACCEL_AXIS_COUNT) - 1U)

/* Per-sensor FFT preprocessing. Mean removal suppresses only the DC bin.
 * It is enabled for acceleration to remove the static gravity projection.
 * Gyroscope and magnetometer defaults preserve their static components. */
#define APP_ACCEL_FFT_REMOVE_MEAN true
#define APP_GYRO_FFT_REMOVE_MEAN  false
#define APP_MAG_FFT_REMOVE_MEAN   false

// Define APP_MAVLINK_ENABLED to include and activate the MAVLink service.
#ifdef APP_MAVLINK_ENABLED
#define APP_MAVLINK_SYSTEM_ID 1U
#define APP_MAVLINK_COMPONENT_ID 191U
#define APP_MAVLINK_HEARTBEAT_INTERVAL_MS 1000U
#define APP_MAVLINK_MAGNETOMETER_INTERVAL_MS 200U
#define APP_MAVLINK_STATUS_INTERVAL_MS 10000U
#define APP_MAVLINK_ATTITUDE_INTERVAL_US 100000U
#define APP_MAVLINK_ATTITUDE_REQUEST_RETRY_MS 5000U
#define APP_MAVLINK_TX_TIMEOUT_MS 100U
#endif

#endif
