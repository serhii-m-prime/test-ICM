#ifndef FFT_PROCESSOR_H
#define FFT_PROCESSOR_H

#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif
#include <stdbool.h>
#include "arm_math.h"

#define FFT_SIZE 256
#define FFT_SHIFT_SIZE 51
#define FFT_SAMPLE_RATE_HZ 1023.5414f

typedef struct {
	float32_t peak_frequency;
	float32_t peak_amplitude;
	float32_t frequency_resolution;
	/* Normalized single-sided amplitudes in input-signal units. */
	float32_t magnitude[FFT_SIZE / 2];
} FFT_Result;

typedef struct {
	arm_rfft_fast_instance_f32 handler;
	float32_t sample_buffer[FFT_SIZE];
	float32_t new_samples[FFT_SHIFT_SIZE];
	float32_t input[FFT_SIZE];
	float32_t output[FFT_SIZE];
	volatile uint16_t new_sample_count;
	volatile uint8_t ready;
	bool remove_mean;
} FFT_Processor;

/* remove_mean controls DC suppression independently for every processor. */
void FFT_Processor_Init(FFT_Processor *processor, bool remove_mean);
void FFT_Processor_AddSample(FFT_Processor *processor, float32_t sample);
uint8_t FFT_Processor_Process(FFT_Processor *processor, FFT_Result *result);

#endif
