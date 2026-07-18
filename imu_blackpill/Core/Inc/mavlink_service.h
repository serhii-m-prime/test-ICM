#ifndef MAVLINK_SERVICE_H
#define MAVLINK_SERVICE_H

#include "stm32f4xx_hal.h"
#include "imu_sensor.h"

#define MAVLINK_SERVICE_RX_BUFFER_SIZE 256U

typedef struct {
	UART_HandleTypeDef *uart;
	uint32_t heartbeat_time;
	uint32_t magnetometer_time;
	uint32_t status_time;
	uint32_t attitude_request_time;
	uint8_t rx_byte;
	uint8_t rx_buffer[MAVLINK_SERVICE_RX_BUFFER_SIZE];
	volatile uint16_t rx_head;
	volatile uint16_t rx_tail;
	volatile uint32_t rx_overflow_count;
	volatile uint32_t uart_error_count;
	volatile float attitude_roll;
	volatile float attitude_pitch;
	volatile float attitude_yaw;
	volatile uint8_t attitude_valid;
} MAVLink_Service;

void MAVLink_Service_Init(MAVLink_Service *service,
		UART_HandleTypeDef *uart);
void MAVLink_Service_Process(MAVLink_Service *service,
		const MagnetometerData *magnetometer);

#endif
