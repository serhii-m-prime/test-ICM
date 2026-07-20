#include "signal_filter_config.h"

const SignalFilterConfig accelerometer_filter_config = {
	.sample_rate_hz = ACCEL_FILTER_SAMPLE_RATE_HZ,
	.ema_alpha = ACCEL_FILTER_EMA_ALPHA,
	.median_window_size = ACCEL_FILTER_MEDIAN_WINDOW_SIZE,
	.band_stop_center_hz = ACCEL_FILTER_BAND_STOP_CENTER_HZ,
	.band_stop_q = ACCEL_FILTER_BAND_STOP_Q,
	.filter_count = ACCEL_FILTER_COUNT,
	.order = ACCEL_FILTER_ORDER
};

const SignalFilterConfig gyroscope_filter_config = {
	.sample_rate_hz = GYRO_FILTER_SAMPLE_RATE_HZ,
	.ema_alpha = GYRO_FILTER_EMA_ALPHA,
	.median_window_size = GYRO_FILTER_MEDIAN_WINDOW_SIZE,
	.band_stop_center_hz = GYRO_FILTER_BAND_STOP_CENTER_HZ,
	.band_stop_q = GYRO_FILTER_BAND_STOP_Q,
	.filter_count = GYRO_FILTER_COUNT,
	.order = GYRO_FILTER_ORDER
};

const SignalFilterConfig magnetometer_filter_config = {
	.sample_rate_hz = MAG_FILTER_SAMPLE_RATE_HZ,
	.ema_alpha = MAG_FILTER_EMA_ALPHA,
	.median_window_size = MAG_FILTER_MEDIAN_WINDOW_SIZE,
	.band_stop_center_hz = MAG_FILTER_BAND_STOP_CENTER_HZ,
	.band_stop_q = MAG_FILTER_BAND_STOP_Q,
	.filter_count = MAG_FILTER_COUNT,
	.order = MAG_FILTER_ORDER
};
