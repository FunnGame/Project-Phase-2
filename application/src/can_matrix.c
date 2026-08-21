/*
 * can_matrix.c
 * Chịu trách nhiệm mã hóa/giải mã payload CAN (Tối đa 8 bytes)
 */
#include "can_matrix.h"

/* Thuật toán mã vòng CRC-8 chuẩn (Đa thức 0x07) */
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

    /* Sắp xếp byte cẩn thận để trùng khớp với logic unpack của ECU2 */
    out_payload[0] = (uint8_t)cmd->steering;
    out_payload[1] = cmd->throttle;
    out_payload[2] = cmd->brake;
    out_payload[3] = cmd->buttons;
    out_payload[4] = cmd->seq;
    out_payload[5] = 0x00U; /* Reserved cho mở rộng tính năng */
    out_payload[6] = 0x00U; /* Reserved */

    /* Gắn mã CRC vào byte cuối cùng (Byte 7) để kiểm tra toàn vẹn trên Bus */
    out_payload[7] = CANMatrix_ComputeCRC8(out_payload, 7U);

    return true;
}

bool CANMatrix_UnpackTelemetry(const uint8_t *payload, CAN_TelemetryStatus_t *out_telemetry) {
    if (payload == NULL || out_telemetry == NULL) return false;

    /*
     * Giải mã bản tin 0x300 từ ECU2
     * Ép kiểu byte theo chuẩn Little-Endian (Intel)
     */
    out_telemetry->speed    = (int16_t)((uint16_t)payload[0] | ((uint16_t)payload[1] << 8));
    out_telemetry->state    = payload[2];
    out_telemetry->faults   = payload[3];
    out_telemetry->throttle = payload[4];
    out_telemetry->brake    = payload[5];

    return true;
}
