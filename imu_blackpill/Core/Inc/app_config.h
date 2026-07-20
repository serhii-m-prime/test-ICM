#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "imu_sensor.h"

/* Only the accelerometer is initialized in the current firmware build. */
#define APP_REQUIRED_SENSOR_COMPONENTS SENSOR_ACCELEROMETER

/* UART stream selection for the filter comparison page. */
#define APP_UART_LEGACY_IMU_STREAM_ENABLED       0
#define APP_UART_ACCEL_COMPARISON_STREAM_ENABLED 1
#define APP_UART_FFT_STREAM_ENABLED              1
#define APP_UART_STATUS_STREAM_ENABLED           1

/* Axis included in message type 4 and processed by FFT: 0=X, 1=Y, 2=Z. */
#define APP_ACCEL_COMPARISON_AXIS 1U

#if APP_ACCEL_COMPARISON_AXIS > 2U
#error "APP_ACCEL_COMPARISON_AXIS must be 0 (X), 1 (Y), or 2 (Z)"
#endif

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
