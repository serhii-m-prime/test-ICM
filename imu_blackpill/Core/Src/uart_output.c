#include "uart_output.h"
#include <stdio.h>
#include <string.h>

static void UART_Output_Send(UART_Output *output, const char *text) {
	HAL_UART_Transmit(output->uart, (uint8_t *) text, strlen(text), HAL_MAX_DELAY);
}

void UART_Output_Init(UART_Output *output, UART_HandleTypeDef *uart) {
	output->uart = uart;
}

void UART_Output_Error(UART_Output *output, const char *description,
		uint8_t requested_components, uint8_t initialized_components) {
	char buffer[192];

	sprintf(buffer, "[0,\"%s\",[%u,%u]]\r\n", description,
			requested_components, initialized_components);
	UART_Output_Send(output, buffer);
}

void UART_Output_Status(UART_Output *output, const char *message) {
	char buffer[128];

	sprintf(buffer, "[3,\"%s\"]\r\n", message);
	UART_Output_Send(output, buffer);
}

void UART_Output_FilterStatus(UART_Output *output, const char *message,
		const SignalFilterConfig *filter_config) {
	char buffer[256];
	char filter_description[112];

	SignalFilterConfig_FormatDescription(filter_config, filter_description,
			sizeof(filter_description));
	sprintf(buffer, "[3,\"%s\",\"%s\"]\r\n", message, filter_description);
	UART_Output_Send(output, buffer);
}

static void UART_Output_FormatVector(char *buffer, const float *vector,
		uint8_t valid) {
	if (valid) {
		sprintf(buffer, "[%.4f,%.4f,%.4f]", vector[0], vector[1], vector[2]);
	} else {
		strcpy(buffer, "null");
	}
}

void UART_Output_SensorData(UART_Output *output,
		const AccelerometerData *accelerometer,
		const GyroscopeData *gyroscope,
		const MagnetometerData *magnetometer) {
	char buffer[256];
	char acceleration[64];
	char angular_rate[64];
	char magnetic_field[64];

	UART_Output_FormatVector(acceleration,
			accelerometer != 0 ? accelerometer->acceleration : 0,
			accelerometer != 0 && accelerometer->valid);
	UART_Output_FormatVector(angular_rate,
			gyroscope != 0 ? gyroscope->angular_rate : 0,
			gyroscope != 0 && gyroscope->valid);
	UART_Output_FormatVector(magnetic_field,
			magnetometer != 0 ? magnetometer->magnetic_field : 0,
			magnetometer != 0 && magnetometer->valid
					&& !magnetometer->overflow);
	sprintf(buffer, "[1,%s,%s,%s]\r\n", acceleration, angular_rate,
			magnetic_field);
	UART_Output_Send(output, buffer);
}

void UART_Output_AccelerometerComparison(UART_Output *output,
		uint32_t time_ms, float raw_value, float handled_value) {
	char buffer[96];

	sprintf(buffer, "[4,[%lu,%.4f,%.4f]]\r\n", (unsigned long) time_ms,
			raw_value, handled_value);
	UART_Output_Send(output, buffer);
}

static void UART_Output_FFTSpectrum(UART_Output *output, char *buffer,
		const FFT_Result *result) {
	for (uint32_t i = 0; i < FFT_SIZE / 2; i++) {
		sprintf(buffer, i == 0 ? "%.4f" : ",%.4f", result->magnitude[i]);
		UART_Output_Send(output, buffer);
	}
}

void UART_Output_FFTComparison(UART_Output *output,
		const FFT_Result *raw_result, const FFT_Result *handled_result) {
	char buffer[96];

	/* Keep handled FFT in fields 0..2 and message[2] for index.html compatibility. */
	sprintf(buffer, "[2,[%.4f,%.4f,%.4f,%.4f,%.4f],[",
			handled_result->peak_frequency, handled_result->peak_amplitude,
			handled_result->frequency_resolution, raw_result->peak_frequency,
			raw_result->peak_amplitude);
	UART_Output_Send(output, buffer);

	UART_Output_FFTSpectrum(output, buffer, handled_result);
	UART_Output_Send(output, "],[");
	UART_Output_FFTSpectrum(output, buffer, raw_result);
	UART_Output_Send(output, "]]\r\n");
}
