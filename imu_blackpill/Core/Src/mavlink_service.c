#include "mavlink_service.h"
#include "app_config.h"

#if APP_MAVLINK_ENABLED
#include "common/mavlink.h"
#include <stdio.h>

#define MICROTESLA_TO_GAUSS 0.01f
#define RADIANS_TO_DEGREES 57.2957795f
#define HIGHRES_IMU_MAG_UPDATED_MASK ((1U << 6) | (1U << 7) | (1U << 8))

static MAVLink_Service *active_service;
static mavlink_message_t rx_message;
static mavlink_status_t rx_status;

static void MAVLink_Service_SendMessage(MAVLink_Service *service,
		mavlink_message_t *message) {
	uint8_t tx_buffer[MAVLINK_MAX_PACKET_LEN];
	uint16_t length = mavlink_msg_to_send_buffer(tx_buffer, message);

	HAL_UART_Transmit(service->uart, tx_buffer, length,
			APP_MAVLINK_TX_TIMEOUT_MS);
}

static void MAVLink_Service_SendHeartbeat(MAVLink_Service *service) {
	mavlink_message_t message;

	mavlink_msg_heartbeat_pack(APP_MAVLINK_SYSTEM_ID,
			APP_MAVLINK_COMPONENT_ID, &message, MAV_TYPE_ONBOARD_CONTROLLER,
			MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
	MAVLink_Service_SendMessage(service, &message);
}

static void MAVLink_Service_RequestAttitude(MAVLink_Service *service) {
	mavlink_message_t message;

	mavlink_msg_command_long_pack(APP_MAVLINK_SYSTEM_ID,
			APP_MAVLINK_COMPONENT_ID, &message, APP_MAVLINK_SYSTEM_ID,
			MAV_COMP_ID_AUTOPILOT1, MAV_CMD_SET_MESSAGE_INTERVAL, 0,
			MAVLINK_MSG_ID_ATTITUDE, APP_MAVLINK_ATTITUDE_INTERVAL_US,
			0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	MAVLink_Service_SendMessage(service, &message);
}

static void MAVLink_Service_SendMagnetometer(MAVLink_Service *service,
		const MagnetometerData *magnetometer) {
	mavlink_message_t message;

	mavlink_msg_highres_imu_pack(APP_MAVLINK_SYSTEM_ID,
			APP_MAVLINK_COMPONENT_ID, &message,
			(uint64_t) HAL_GetTick() * 1000U, 0.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 0.0f,
			magnetometer->magnetic_field[0] * MICROTESLA_TO_GAUSS,
			magnetometer->magnetic_field[1] * MICROTESLA_TO_GAUSS,
			magnetometer->magnetic_field[2] * MICROTESLA_TO_GAUSS,
			0.0f, 0.0f, 0.0f, 0.0f, HIGHRES_IMU_MAG_UPDATED_MASK, 0);
	MAVLink_Service_SendMessage(service, &message);
}

static void MAVLink_Service_SendStatus(MAVLink_Service *service) {
	mavlink_message_t message;
	char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = { 0 };

	if (service->attitude_valid) {
		snprintf(text, sizeof(text), "ATT r=%.1f p=%.1f y=%.1f deg",
				service->attitude_roll * RADIANS_TO_DEGREES,
				service->attitude_pitch * RADIANS_TO_DEGREES,
				service->attitude_yaw * RADIANS_TO_DEGREES);
	} else {
		snprintf(text, sizeof(text), "Blackpill: waiting for ATTITUDE");
	}

	mavlink_msg_statustext_pack(APP_MAVLINK_SYSTEM_ID,
			APP_MAVLINK_COMPONENT_ID, &message, MAV_SEVERITY_INFO, text, 0, 0);
	MAVLink_Service_SendMessage(service, &message);
}

static void MAVLink_Service_HandleMessage(MAVLink_Service *service,
		const mavlink_message_t *message) {
	if (message->msgid == MAVLINK_MSG_ID_ATTITUDE
			&& message->sysid == APP_MAVLINK_SYSTEM_ID
			&& message->compid == MAV_COMP_ID_AUTOPILOT1) {
		mavlink_attitude_t attitude;

		mavlink_msg_attitude_decode(message, &attitude);
		service->attitude_roll = attitude.roll;
		service->attitude_pitch = attitude.pitch;
		service->attitude_yaw = attitude.yaw;
		service->attitude_valid = 1;
	}
}

static void MAVLink_Service_ProcessReceivedBytes(MAVLink_Service *service) {
	while (service->rx_tail != service->rx_head) {
		uint8_t byte = service->rx_buffer[service->rx_tail];
		service->rx_tail = (service->rx_tail + 1U)
				% MAVLINK_SERVICE_RX_BUFFER_SIZE;

		if (mavlink_parse_char(MAVLINK_COMM_0, byte, &rx_message,
				&rx_status)) {
			MAVLink_Service_HandleMessage(service, &rx_message);
		}
	}
}
#endif

void MAVLink_Service_Init(MAVLink_Service *service,
		UART_HandleTypeDef *uart) {
	service->uart = uart;
	service->heartbeat_time = HAL_GetTick();
	service->magnetometer_time = HAL_GetTick();
	service->status_time = HAL_GetTick();
	service->attitude_request_time = 0;
	service->rx_head = 0;
	service->rx_tail = 0;
	service->rx_overflow_count = 0;
	service->uart_error_count = 0;
	service->attitude_valid = 0;

#if APP_MAVLINK_ENABLED
	active_service = service;
	HAL_UART_Receive_IT(service->uart, &service->rx_byte, 1);
#endif
}

void MAVLink_Service_Process(MAVLink_Service *service,
		const MagnetometerData *magnetometer) {
#if APP_MAVLINK_ENABLED
	uint32_t now = HAL_GetTick();
	MAVLink_Service_ProcessReceivedBytes(service);

	if ((now - service->heartbeat_time)
			>= APP_MAVLINK_HEARTBEAT_INTERVAL_MS) {
		service->heartbeat_time = now;
		MAVLink_Service_SendHeartbeat(service);
	}

	if (!service->attitude_valid
			&& (now - service->attitude_request_time)
					>= APP_MAVLINK_ATTITUDE_REQUEST_RETRY_MS) {
		service->attitude_request_time = now;
		MAVLink_Service_RequestAttitude(service);
	}

	if (magnetometer != 0 && magnetometer->valid && !magnetometer->overflow
			&& (now - service->magnetometer_time)
					>= APP_MAVLINK_MAGNETOMETER_INTERVAL_MS) {
		service->magnetometer_time = now;
		MAVLink_Service_SendMagnetometer(service, magnetometer);
	}

	if ((now - service->status_time) >= APP_MAVLINK_STATUS_INTERVAL_MS) {
		service->status_time = now;
		MAVLink_Service_SendStatus(service);
	}
#else
	(void) service;
	(void) magnetometer;
#endif
}

#if APP_MAVLINK_ENABLED
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart) {
	uint16_t next_head;

	if (active_service == 0 || uart != active_service->uart) {
		return;
	}

	next_head = (active_service->rx_head + 1U)
			% MAVLINK_SERVICE_RX_BUFFER_SIZE;
	if (next_head != active_service->rx_tail) {
		active_service->rx_buffer[active_service->rx_head] =
				active_service->rx_byte;
		active_service->rx_head = next_head;
	} else {
		active_service->rx_overflow_count++;
	}

	HAL_UART_Receive_IT(active_service->uart, &active_service->rx_byte, 1);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) {
	if (active_service != 0 && uart == active_service->uart) {
		active_service->uart_error_count++;
		HAL_UART_Receive_IT(active_service->uart, &active_service->rx_byte, 1);
	}
}
#endif
