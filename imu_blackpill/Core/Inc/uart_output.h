#ifndef UART_OUTPUT_H
#define UART_OUTPUT_H

#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "imu_sensor.h"
#include "fft_processor.h"
#include "signal_filter.h"

#define UART_OUTPUT_TX_BUFFER_SIZE      8192U
#define UART_OUTPUT_MESSAGE_BUFFER_SIZE 4096U

typedef struct {
	UART_HandleTypeDef *uart;
	uint8_t tx_buffer[UART_OUTPUT_TX_BUFFER_SIZE];
	char message_buffer[UART_OUTPUT_MESSAGE_BUFFER_SIZE];
	volatile uint16_t head;
	volatile uint16_t tail;
	volatile uint16_t dma_length;
	volatile bool dma_active;
	volatile uint32_t dropped_messages;
} UART_Output;

void UART_Output_Init(UART_Output *output, UART_HandleTypeDef *uart);
void UART_Output_Process(UART_Output *output);
void UART_Output_TxComplete(UART_Output *output);
void UART_Output_TxError(UART_Output *output);
uint32_t UART_Output_GetDroppedMessageCount(const UART_Output *output);
void UART_Output_Error(UART_Output *output, const char *description,
		uint8_t requested_components, uint8_t initialized_components);
void UART_Output_Status(UART_Output *output, const char *message);
void UART_Output_FilterStatus(UART_Output *output, const char *message,
		const SignalFilterConfig *filter_config);
void UART_Output_SensorData(UART_Output *output,
		const AccelerometerData *accelerometer,
		const GyroscopeData *gyroscope,
		const MagnetometerData *magnetometer);
/* [4,[time_ms,[raw_x,raw_y,raw_z],[handled_x,handled_y,handled_z]]] */
void UART_Output_AccelerometerComparison(UART_Output *output,
		uint32_t time_ms, const float raw_value[3],
		const float handled_value[3]);
/* [2,axis,[handled_peak_hz,handled_amp,hz_per_bin,raw_peak_hz,raw_amp],...]
 * axis: 0=X, 1=Y, 2=Z; followed by handled and raw magnitude arrays. */
void UART_Output_FFTComparison(UART_Output *output,
		uint8_t axis, const FFT_Result *raw_result,
		const FFT_Result *handled_result);

#endif
