#ifndef UART_OUTPUT_H
#define UART_OUTPUT_H

#include "stm32f4xx_hal.h"
#include "imu_sensor.h"
#include "fft_processor.h"
#include "signal_filter.h"

typedef struct {
	UART_HandleTypeDef *uart;
} UART_Output;

void UART_Output_Init(UART_Output *output, UART_HandleTypeDef *uart);
void UART_Output_Error(UART_Output *output, const char *description,
		uint8_t requested_components, uint8_t initialized_components);
void UART_Output_Status(UART_Output *output, const char *message);
void UART_Output_FilterStatus(UART_Output *output, const char *message,
		const SignalFilterConfig *filter_config);
void UART_Output_SensorData(UART_Output *output,
		const AccelerometerData *accelerometer,
		const GyroscopeData *gyroscope,
		const MagnetometerData *magnetometer);
void UART_Output_AccelerometerComparison(UART_Output *output,
		uint32_t time_ms, float raw_value, float handled_value);
void UART_Output_FFTComparison(UART_Output *output,
		const FFT_Result *raw_result, const FFT_Result *handled_result);

#endif
