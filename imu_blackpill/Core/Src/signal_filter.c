#include "signal_filter.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define SIGNAL_FILTER_PI 3.14159265358979323846f

static bool SignalFilterConfig_IsValid(const SignalFilterConfig *config) {
	uint8_t used_filters = 0U;

	if (config == 0 || config->filter_count > SIGNAL_FILTER_MAX_CHAIN_LENGTH
			|| config->sample_rate_hz <= 0.0f) {
		return false;
	}

	for (uint8_t i = 0U; i < config->filter_count; i++) {
		SignalFilterType type = config->order[i];
		uint8_t filter_bit;

		if (type == SIGNAL_FILTER_NONE || type > SIGNAL_FILTER_BAND_STOP) {
			return false;
		}

		filter_bit = (uint8_t) (1U << (uint8_t) type);
		if ((used_filters & filter_bit) != 0U) {
			return false;
		}
		used_filters |= filter_bit;
	}

	if ((used_filters & (1U << SIGNAL_FILTER_EMA)) != 0U
			&& (config->ema_alpha <= 0.0f || config->ema_alpha > 1.0f)) {
		return false;
	}

	if ((used_filters & (1U << SIGNAL_FILTER_MEDIAN)) != 0U
			&& (config->median_window_size == 0U
					|| config->median_window_size > SIGNAL_FILTER_MEDIAN_MAX_WINDOW
					|| (config->median_window_size & 1U) == 0U)) {
		return false;
	}

	if ((used_filters & (1U << SIGNAL_FILTER_BAND_STOP)) != 0U
			&& (config->band_stop_center_hz <= 0.0f
					|| config->band_stop_center_hz >= config->sample_rate_hz * 0.5f
					|| config->band_stop_q <= 0.0f)) {
		return false;
	}

	return true;
}

static void SignalFilterBandStop_Init(SignalFilterChain *chain) {
	float omega = 2.0f * SIGNAL_FILTER_PI
			* chain->config->band_stop_center_hz
			/ chain->config->sample_rate_hz;
	float cosine = cosf(omega);
	float alpha = sinf(omega) / (2.0f * chain->config->band_stop_q);
	float inverse_a0 = 1.0f / (1.0f + alpha);

	chain->band_stop.b0 = inverse_a0;
	chain->band_stop.b1 = -2.0f * cosine * inverse_a0;
	chain->band_stop.b2 = inverse_a0;
	chain->band_stop.a1 = -2.0f * cosine * inverse_a0;
	chain->band_stop.a2 = (1.0f - alpha) * inverse_a0;
}

static float SignalFilterEma_Process(SignalFilterChain *chain, float input) {
	if (!chain->ema.initialized) {
		chain->ema.output = input;
		chain->ema.initialized = true;
	} else {
		chain->ema.output = chain->config->ema_alpha * input
				+ (1.0f - chain->config->ema_alpha) * chain->ema.output;
	}

	return chain->ema.output;
}

static float SignalFilterMedian_Process(SignalFilterChain *chain, float input) {
	float sorted[SIGNAL_FILTER_MEDIAN_MAX_WINDOW];
	uint8_t sample_count;

	chain->median.samples[chain->median.index] = input;
	chain->median.index++;
	if (chain->median.index >= chain->config->median_window_size) {
		chain->median.index = 0U;
	}
	if (chain->median.count < chain->config->median_window_size) {
		chain->median.count++;
	}

	sample_count = chain->median.count;
	memcpy(sorted, chain->median.samples, sample_count * sizeof(float));

	for (uint8_t i = 1U; i < sample_count; i++) {
		float value = sorted[i];
		uint8_t position = i;

		while (position > 0U && sorted[position - 1U] > value) {
			sorted[position] = sorted[position - 1U];
			position--;
		}
		sorted[position] = value;
	}

	return sorted[sample_count / 2U];
}

static float SignalFilterBandStop_Process(SignalFilterChain *chain,
		float input) {
	if (!chain->band_stop.initialized) {
		chain->band_stop.x1 = input;
		chain->band_stop.x2 = input;
		chain->band_stop.y1 = input;
		chain->band_stop.y2 = input;
		chain->band_stop.initialized = true;
		return input;
	}

	float output = chain->band_stop.b0 * input
			+ chain->band_stop.b1 * chain->band_stop.x1
			+ chain->band_stop.b2 * chain->band_stop.x2
			- chain->band_stop.a1 * chain->band_stop.y1
			- chain->band_stop.a2 * chain->band_stop.y2;

	chain->band_stop.x2 = chain->band_stop.x1;
	chain->band_stop.x1 = input;
	chain->band_stop.y2 = chain->band_stop.y1;
	chain->band_stop.y1 = output;

	return output;
}

bool SignalFilterChain_Init(SignalFilterChain *chain,
		const SignalFilterConfig *config) {
	if (chain == 0 || !SignalFilterConfig_IsValid(config)) {
		return false;
	}

	memset(chain, 0, sizeof(*chain));
	chain->config = config;
	SignalFilterBandStop_Init(chain);
	return true;
}

void SignalFilterChain_Reset(SignalFilterChain *chain) {
	const SignalFilterConfig *config;

	if (chain == 0) {
		return;
	}

	config = chain->config;
	memset(chain, 0, sizeof(*chain));
	chain->config = config;
	if (config != 0) {
		SignalFilterBandStop_Init(chain);
	}
}

float SignalFilterChain_Process(SignalFilterChain *chain, float input) {
	float output = input;

	if (chain == 0 || chain->config == 0) {
		return input;
	}

	for (uint8_t i = 0U; i < chain->config->filter_count; i++) {
		switch (chain->config->order[i]) {
		case SIGNAL_FILTER_EMA:
			output = SignalFilterEma_Process(chain, output);
			break;
		case SIGNAL_FILTER_MEDIAN:
			output = SignalFilterMedian_Process(chain, output);
			break;
		case SIGNAL_FILTER_BAND_STOP:
			output = SignalFilterBandStop_Process(chain, output);
			break;
		default:
			break;
		}
	}

	return output;
}

void SignalFilterConfig_FormatDescription(const SignalFilterConfig *config,
		char *buffer, size_t buffer_size) {
	size_t used = 0U;

	if (buffer == 0 || buffer_size == 0U) {
		return;
	}
	buffer[0] = '\0';

	if (config == 0 || config->filter_count == 0U) {
		(void) snprintf(buffer, buffer_size, "RAW");
		return;
	}

	for (uint8_t i = 0U; i < config->filter_count && used < buffer_size; i++) {
		int written;
		const char *separator = i == 0U ? "" : " > ";

		switch (config->order[i]) {
		case SIGNAL_FILTER_EMA:
			written = snprintf(buffer + used, buffer_size - used,
					"%sEMA(alpha=%.3f)", separator, config->ema_alpha);
			break;
		case SIGNAL_FILTER_MEDIAN:
			written = snprintf(buffer + used, buffer_size - used,
					"%sMEDIAN(%u)", separator,
					(unsigned int) config->median_window_size);
			break;
		case SIGNAL_FILTER_BAND_STOP:
			written = snprintf(buffer + used, buffer_size - used,
					"%sBAND_STOP(%.1fHz,Q=%.1f)", separator,
					config->band_stop_center_hz, config->band_stop_q);
			break;
		default:
			written = snprintf(buffer + used, buffer_size - used,
					"%sUNKNOWN", separator);
			break;
		}

		if (written < 0) {
			buffer[0] = '\0';
			return;
		}
		if ((size_t) written >= buffer_size - used) {
			used = buffer_size;
		} else {
			used += (size_t) written;
		}
	}
}
