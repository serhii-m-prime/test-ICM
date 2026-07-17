#include "fft_processor.h"
#include <math.h>
#include <string.h>

void FFT_Processor_Init(FFT_Processor *processor) {
	memset(processor, 0, sizeof(*processor));
	arm_rfft_fast_init_f32(&processor->handler, FFT_SIZE);
}

void FFT_Processor_AddSample(FFT_Processor *processor, float32_t sample) {
	processor->new_samples[processor->new_sample_count] = sample;
	processor->new_sample_count++;

	if (processor->new_sample_count >= FFT_SHIFT_SIZE) {
		if (!processor->ready) {
			memmove(processor->sample_buffer, processor->sample_buffer + FFT_SHIFT_SIZE,
					(FFT_SIZE - FFT_SHIFT_SIZE) * sizeof(float32_t));
			memcpy(processor->sample_buffer + FFT_SIZE - FFT_SHIFT_SIZE,
					processor->new_samples, FFT_SHIFT_SIZE * sizeof(float32_t));
			memcpy(processor->input, processor->sample_buffer, FFT_SIZE * sizeof(float32_t));
			processor->ready = 1;
		}
		processor->new_sample_count = 0;
	}
}

uint8_t FFT_Processor_Process(FFT_Processor *processor, FFT_Result *result) {
	float32_t max_value = 0.0f;
	uint32_t max_index = 0;

	if (!processor->ready) {
		return 0;
	}

	arm_rfft_fast_f32(&processor->handler, processor->input, processor->output, 0);
	result->magnitude[0] = fabsf(processor->output[0]);

	for (uint32_t i = 1; i < FFT_SIZE / 2; i++) {
		float32_t real = processor->output[2 * i];
		float32_t imaginary = processor->output[2 * i + 1];
		result->magnitude[i] = sqrtf(real * real + imaginary * imaginary);
	}

	for (uint32_t i = 2; i < FFT_SIZE / 2; i++) {
		if (result->magnitude[i] > max_value) {
			max_value = result->magnitude[i];
			max_index = i;
		}
	}

	result->frequency_resolution = FFT_SAMPLE_RATE_HZ / (float32_t) FFT_SIZE;
	result->peak_frequency = (float32_t) max_index
			* result->frequency_resolution;
	result->peak_amplitude = max_value;
	processor->ready = 0;

	return 1;
}
