/*
 * can_matrix.c
 *
 * Author: trong
 */

#include "can_matrix.h"
#include <string.h>

uint8_t CANMatrix_ComputeCRC8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00U;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8U; ++bit) {
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

bool CANMatrix_PackControlCmd(const CAN_ControlCmd_t *cmd, uint8_t *out_payload) {
    if (cmd == NULL || out_payload == NULL) return false;

    out_payload[0] = (uint8_t)cmd->throttle;
    out_payload[1] = (uint8_t)cmd->steering;
    out_payload[2] = cmd->mode;
    out_payload[3] = cmd->flags;
    out_payload[4] = cmd->seq;
    out_payload[5] = 0x00U; /* Reserved */
    out_payload[6] = 0x00U; /* Reserved */
    out_payload[7] = CANMatrix_ComputeCRC8(out_payload, 7U);

    return true;
}

bool CANMatrix_UnpackTelemetry(const uint8_t *payload, CAN_TelemetryStatus_t *out_telemetry) {
    if (payload == NULL || out_telemetry == NULL) return false;

    out_telemetry->speed_rpm       = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    out_telemetry->acc_active      = payload[2];
    out_telemetry->aeb_active      = payload[3];
    out_telemetry->battery_percent = payload[4];

    return true;
}
