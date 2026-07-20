#ifndef SIGNAL_FILTER_CONFIG_H
#define SIGNAL_FILTER_CONFIG_H

#include "signal_filter.h"

#define ACCEL_FILTER_SAMPLE_RATE_HZ       1023.5414f
#define ACCEL_FILTER_EMA_ALPHA            0.20f
#define ACCEL_FILTER_MEDIAN_WINDOW_SIZE   5U
#define ACCEL_FILTER_BAND_STOP_CENTER_HZ  4.0f
#define ACCEL_FILTER_BAND_STOP_Q          20.0f
#define ACCEL_FILTER_COUNT                3U
#define ACCEL_FILTER_ORDER                \
	{ SIGNAL_FILTER_MEDIAN, SIGNAL_FILTER_BAND_STOP, SIGNAL_FILTER_EMA }

#define GYRO_FILTER_SAMPLE_RATE_HZ        1023.5414f
#define GYRO_FILTER_EMA_ALPHA             0.10f
#define GYRO_FILTER_MEDIAN_WINDOW_SIZE    5U
#define GYRO_FILTER_BAND_STOP_CENTER_HZ   50.0f
#define GYRO_FILTER_BAND_STOP_Q           10.0f
#define GYRO_FILTER_COUNT                 0U
#define GYRO_FILTER_ORDER                 \
	{ SIGNAL_FILTER_NONE, SIGNAL_FILTER_NONE, SIGNAL_FILTER_NONE }

#define MAG_FILTER_SAMPLE_RATE_HZ         100.0f
#define MAG_FILTER_EMA_ALPHA              0.10f
#define MAG_FILTER_MEDIAN_WINDOW_SIZE     5U
#define MAG_FILTER_BAND_STOP_CENTER_HZ    25.0f
#define MAG_FILTER_BAND_STOP_Q            10.0f
#define MAG_FILTER_COUNT                  0U
#define MAG_FILTER_ORDER                  \
	{ SIGNAL_FILTER_NONE, SIGNAL_FILTER_NONE, SIGNAL_FILTER_NONE }

extern const SignalFilterConfig accelerometer_filter_config;
extern const SignalFilterConfig gyroscope_filter_config;
extern const SignalFilterConfig magnetometer_filter_config;

#endif
