/*
 * can_matrix.h
 * Tối ưu hóa: Đồng bộ với rf_protocol (Control-Station) và chuẩn bị dữ liệu cho ECU2.
 */
#ifndef CAN_MATRIX_H
#define CAN_MATRIX_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Cấu hình CAN ID chuẩn 11-bit */
#define CAN_ID_CONTROL_CMD       0x100U
#define CAN_ID_SENSOR_DISTANCE   0x200U
#define CAN_ID_TELEMETRY_STATUS  0x300U
#define CAN_ID_SYSTEM_POST       0x400U

/*
 * Struct Lệnh Điều Khiển (Gateway -> ECU2)
 * Đã đồng bộ kiểu dữ liệu với rf_control_frame_t
 */
typedef struct {
    int8_t  steering; /* -100..+100 (Trái..Phải) */
    uint8_t throttle; /* 0..100 (Độ lớn ga) */
    uint8_t brake;    /* 0..100 (Độ lớn phanh) */
    uint8_t buttons;  /* bit0 = reverse, bit1 = armed */
    uint8_t seq;      /* Bộ đếm Sequence */
} CAN_ControlCmd_t;

/*
 * Struct Trạng Thái (ECU2 -> Gateway)
 * Cung cấp đủ thông số để Gateway đóng gói vào rf_telemetry_frame_t
 */
typedef struct {
    int16_t speed;    /* Vận tốc thực tế (mm/s) */
    uint8_t state;    /* Trạng thái Xe & AEB */
    uint8_t faults;   /* Mã lỗi (Faults) */
    uint8_t throttle; /* Ga đang áp dụng (Applied) */
    uint8_t brake;    /* Phanh đang áp dụng (Applied) */
} CAN_TelemetryStatus_t;

uint8_t CANMatrix_ComputeCRC8(const uint8_t *data, size_t len);
bool    CANMatrix_PackControlCmd(const CAN_ControlCmd_t *cmd, uint8_t *out_payload);
bool    CANMatrix_UnpackTelemetry(const uint8_t *payload, CAN_TelemetryStatus_t *out_telemetry);

#endif /* CAN_MATRIX_H */
