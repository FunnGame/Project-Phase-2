/*
 * can_matrix.h
 * Author: trong
 */
#ifndef CAN_MATRIX_H
#define CAN_MATRIX_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define CAN_ID_CONTROL_CMD       0x100U
#define CAN_ID_SENSOR_DISTANCE   0x200U
#define CAN_ID_TELEMETRY_STATUS  0x300U
#define CAN_ID_SYSTEM_POST       0x400U

/* Struct đã được cập nhật đồng bộ với RF Protocol mới */
typedef struct {
    int8_t  throttle;
    int8_t  steering;
    uint8_t brake;    /* Thay cho mode */
    uint8_t buttons;  /* Thay cho flags */
    uint8_t seq;
} CAN_ControlCmd_t;

typedef struct {
    uint16_t speed_rpm;
    uint8_t  acc_active;
    uint8_t  aeb_active;
    uint8_t  battery_percent;
} CAN_TelemetryStatus_t;

uint8_t CANMatrix_ComputeCRC8(const uint8_t *data, size_t len);
bool    CANMatrix_PackControlCmd(const CAN_ControlCmd_t *cmd, uint8_t *out_payload);
bool    CANMatrix_UnpackTelemetry(const uint8_t *payload, CAN_TelemetryStatus_t *out_telemetry);

#endif /* CAN_MATRIX_H */
