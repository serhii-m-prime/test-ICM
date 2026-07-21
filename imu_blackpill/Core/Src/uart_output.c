#include "uart_output.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

_Static_assert(UART_OUTPUT_TX_BUFFER_SIZE > 1U,
		"UART TX ring buffer must contain at least two bytes");
_Static_assert(UART_OUTPUT_TX_BUFFER_SIZE <= UINT16_MAX,
		"UART TX ring buffer indices are uint16_t");
_Static_assert(UART_OUTPUT_MESSAGE_BUFFER_SIZE <= UINT16_MAX,
		"UART message length is uint16_t");

static uint16_t UART_Output_FreeSpace(const UART_Output *output) {
	if (output->head >= output->tail) {
		return (uint16_t) (UART_OUTPUT_TX_BUFFER_SIZE
				- (output->head - output->tail) - 1U);
	}

	return (uint16_t) (output->tail - output->head - 1U);
}

/* Must be called while interrupts are disabled or from the TX callback. */
static void UART_Output_StartDma(UART_Output *output) {
	uint16_t length;

	if (output->dma_active || output->head == output->tail) {
		return;
	}

	if (output->head > output->tail) {
		length = (uint16_t) (output->head - output->tail);
	} else {
		length = (uint16_t) (UART_OUTPUT_TX_BUFFER_SIZE - output->tail);
	}

	output->dma_length = length;
	output->dma_active = true;
	__DMB();

	if (HAL_UART_Transmit_DMA(output->uart, &output->tx_buffer[output->tail],
			length) != HAL_OK) {
		output->dma_length = 0U;
		output->dma_active = false;
	}
}

static bool UART_Output_Queue(UART_Output *output, const char *data,
		uint16_t length) {
	uint32_t interrupt_state;
	uint16_t first_part;

	if (output == 0 || data == 0) {
		return false;
	}
	if (length == 0U) {
		return true;
	}
	if (length >= UART_OUTPUT_TX_BUFFER_SIZE) {
		output->dropped_messages++;
		return false;
	}

	interrupt_state = __get_PRIMASK();
	__disable_irq();

	if (UART_Output_FreeSpace(output) < length) {
		output->dropped_messages++;
		if (interrupt_state == 0U) {
			__enable_irq();
		}
		return false;
	}

	first_part = (uint16_t) (UART_OUTPUT_TX_BUFFER_SIZE - output->head);
	if (first_part > length) {
		first_part = length;
	}

	memcpy(&output->tx_buffer[output->head], data, first_part);
	if (length > first_part) {
		memcpy(output->tx_buffer, data + first_part, length - first_part);
	}
	output->head = (uint16_t) ((output->head + length)
			% UART_OUTPUT_TX_BUFFER_SIZE);

	UART_Output_StartDma(output);

	if (interrupt_state == 0U) {
		__enable_irq();
	}
	return true;
}

static bool UART_Output_Send(UART_Output *output, const char *text) {
	size_t length;

	if (output == 0 || text == 0) {
		return false;
	}

	length = strlen(text);
	if (length > UINT16_MAX) {
		output->dropped_messages++;
		return false;
	}

	return UART_Output_Queue(output, text, (uint16_t) length);
}

static bool UART_Output_AppendFormat(char *buffer, size_t buffer_size,
		size_t *used, const char *format, ...) {
	va_list arguments;
	int written;

	if (*used >= buffer_size) {
		return false;
	}

	va_start(arguments, format);
	written = vsnprintf(buffer + *used, buffer_size - *used, format, arguments);
	va_end(arguments);

	if (written < 0 || (size_t) written >= buffer_size - *used) {
		*used = buffer_size;
		return false;
	}

	*used += (size_t) written;
	return true;
}

void UART_Output_Init(UART_Output *output, UART_HandleTypeDef *uart) {
	memset(output, 0, sizeof(*output));
	output->uart = uart;
}

void UART_Output_Process(UART_Output *output) {
	uint32_t interrupt_state;

	if (output == 0 || output->uart == 0) {
		return;
	}

	interrupt_state = __get_PRIMASK();
	__disable_irq();
	UART_Output_StartDma(output);
	if (interrupt_state == 0U) {
		__enable_irq();
	}
}

void UART_Output_TxComplete(UART_Output *output) {
	if (output == 0 || !output->dma_active) {
		return;
	}

	output->tail = (uint16_t) ((output->tail + output->dma_length)
			% UART_OUTPUT_TX_BUFFER_SIZE);
	output->dma_length = 0U;
	output->dma_active = false;
	UART_Output_StartDma(output);
}

void UART_Output_TxError(UART_Output *output) {
	if (output == 0) {
		return;
	}

	/* Discard the damaged queue and terminate a potentially partial JSON line. */
	output->head = 2U;
	output->tail = 0U;
	output->dma_length = 0U;
	output->dma_active = false;
	output->tx_buffer[0] = '\r';
	output->tx_buffer[1] = '\n';
	output->dropped_messages++;
	UART_Output_StartDma(output);
}

uint32_t UART_Output_GetDroppedMessageCount(const UART_Output *output) {
	return output != 0 ? output->dropped_messages : 0U;
}

void UART_Output_Error(UART_Output *output, const char *description,
		uint8_t requested_components, uint8_t initialized_components) {
	char buffer[192];

	snprintf(buffer, sizeof(buffer), "[0,\"%s\",[%u,%u]]\r\n", description,
			requested_components, initialized_components);
	UART_Output_Send(output, buffer);
}

void UART_Output_Status(UART_Output *output, const char *message) {
	char buffer[128];

	snprintf(buffer, sizeof(buffer), "[3,\"%s\"]\r\n", message);
	UART_Output_Send(output, buffer);
}

void UART_Output_FilterStatus(UART_Output *output, const char *message,
		const SignalFilterConfig *filter_config) {
	char buffer[256];
	char filter_description[112];

	SignalFilterConfig_FormatDescription(filter_config, filter_description,
			sizeof(filter_description));
	snprintf(buffer, sizeof(buffer), "[3,\"%s\",\"%s\"]\r\n", message,
			filter_description);
	UART_Output_Send(output, buffer);
}

static void UART_Output_FormatVector(char *buffer, size_t buffer_size,
		const float *vector, uint8_t valid) {
	if (valid) {
		snprintf(buffer, buffer_size, "[%.4f,%.4f,%.4f]", vector[0], vector[1],
				vector[2]);
	} else {
		snprintf(buffer, buffer_size, "null");
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

	UART_Output_FormatVector(acceleration, sizeof(acceleration),
			accelerometer != 0 ? accelerometer->acceleration : 0,
			accelerometer != 0 && accelerometer->valid);
	UART_Output_FormatVector(angular_rate, sizeof(angular_rate),
			gyroscope != 0 ? gyroscope->angular_rate : 0,
			gyroscope != 0 && gyroscope->valid);
	UART_Output_FormatVector(magnetic_field, sizeof(magnetic_field),
			magnetometer != 0 ? magnetometer->magnetic_field : 0,
			magnetometer != 0 && magnetometer->valid && !magnetometer->overflow);
	snprintf(buffer, sizeof(buffer), "[1,%s,%s,%s]\r\n", acceleration,
			angular_rate, magnetic_field);
	UART_Output_Send(output, buffer);
}

void UART_Output_AccelerometerComparison(UART_Output *output,
		uint32_t time_ms, const float raw_value[3],
		const float handled_value[3]) {
	char buffer[160];

	snprintf(buffer, sizeof(buffer),
			"[4,[%lu,[%.4f,%.4f,%.4f],[%.4f,%.4f,%.4f]]]\r\n",
			(unsigned long) time_ms,
			raw_value[0], raw_value[1], raw_value[2],
			handled_value[0], handled_value[1], handled_value[2]);
	UART_Output_Send(output, buffer);
}

void UART_Output_FFTComparison(UART_Output *output,
		uint8_t axis, const FFT_Result *raw_result,
		const FFT_Result *handled_result) {
	size_t used = 0U;
	bool valid;

	valid = UART_Output_AppendFormat(output->message_buffer,
			UART_OUTPUT_MESSAGE_BUFFER_SIZE, &used,
			"[2,%u,[%.4f,%.4f,%.4f,%.4f,%.4f],[", axis,
			handled_result->peak_frequency, handled_result->peak_amplitude,
			handled_result->frequency_resolution, raw_result->peak_frequency,
			raw_result->peak_amplitude);

	for (uint32_t i = 0U; valid && i < FFT_SIZE / 2U; i++) {
		valid = UART_Output_AppendFormat(output->message_buffer,
				UART_OUTPUT_MESSAGE_BUFFER_SIZE, &used,
				i == 0U ? "%.4f" : ",%.4f", handled_result->magnitude[i]);
	}

	if (valid) {
		valid = UART_Output_AppendFormat(output->message_buffer,
				UART_OUTPUT_MESSAGE_BUFFER_SIZE, &used, "],[");
	}

	for (uint32_t i = 0U; valid && i < FFT_SIZE / 2U; i++) {
		valid = UART_Output_AppendFormat(output->message_buffer,
				UART_OUTPUT_MESSAGE_BUFFER_SIZE, &used,
				i == 0U ? "%.4f" : ",%.4f", raw_result->magnitude[i]);
	}

	if (valid) {
		valid = UART_Output_AppendFormat(output->message_buffer,
				UART_OUTPUT_MESSAGE_BUFFER_SIZE, &used, "]]\r\n");
	}

	if (!valid || used > UINT16_MAX) {
		output->dropped_messages++;
		return;
	}

	UART_Output_Queue(output, output->message_buffer, (uint16_t) used);
}
