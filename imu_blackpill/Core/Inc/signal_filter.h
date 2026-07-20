#ifndef SIGNAL_FILTER_H
#define SIGNAL_FILTER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SIGNAL_FILTER_MAX_CHAIN_LENGTH 3U
#define SIGNAL_FILTER_MEDIAN_MAX_WINDOW 9U

typedef enum {
	SIGNAL_FILTER_NONE = 0,
	SIGNAL_FILTER_EMA,
	SIGNAL_FILTER_MEDIAN,
	SIGNAL_FILTER_BAND_STOP
} SignalFilterType;

typedef struct {
	float sample_rate_hz;
	float ema_alpha;
	uint8_t median_window_size;
	float band_stop_center_hz;
	float band_stop_q;
	uint8_t filter_count;
	SignalFilterType order[SIGNAL_FILTER_MAX_CHAIN_LENGTH];
} SignalFilterConfig;

typedef struct {
	float output;
	bool initialized;
} SignalFilterEmaState;

typedef struct {
	float samples[SIGNAL_FILTER_MEDIAN_MAX_WINDOW];
	uint8_t index;
	uint8_t count;
} SignalFilterMedianState;

typedef struct {
	float b0;
	float b1;
	float b2;
	float a1;
	float a2;
	float x1;
	float x2;
	float y1;
	float y2;
	bool initialized;
} SignalFilterBandStopState;

typedef struct {
	const SignalFilterConfig *config;
	SignalFilterEmaState ema;
	SignalFilterMedianState median;
	SignalFilterBandStopState band_stop;
} SignalFilterChain;

bool SignalFilterChain_Init(SignalFilterChain *chain,
		const SignalFilterConfig *config);
void SignalFilterChain_Reset(SignalFilterChain *chain);
float SignalFilterChain_Process(SignalFilterChain *chain, float input);
void SignalFilterConfig_FormatDescription(const SignalFilterConfig *config,
		char *buffer, size_t buffer_size);

#endif
