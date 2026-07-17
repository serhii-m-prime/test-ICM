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

void UART_Output_FFTResult(UART_Output *output, const FFT_Result *result) {
	char buffer[96];

	sprintf(buffer, "[2,[%.4f,%.4f,%.4f],[", result->peak_frequency,
			result->peak_amplitude, result->frequency_resolution);
	UART_Output_Send(output, buffer);

	for (uint32_t i = 0; i < FFT_SIZE / 2; i++) {
		sprintf(buffer, i == 0 ? "%.4f" : ",%.4f", result->magnitude[i]);
		UART_Output_Send(output, buffer);
	}

	UART_Output_Send(output, "]]\r\n");
}
