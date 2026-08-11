/*
 * ecu2_can_app.h
 *
 * Description: Application layer for handling CAN Rx/Tx on ECU2 (Motor Control).
 * Depends on ECU3's CAN Matrix specifications and ECU2's control APIs.
 */

#ifndef ECU2_CAN_APP_H
#define ECU2_CAN_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "drv_can.h"

/* --- CAN ID Defines (Theo tài liệu can_matrix.md) --- */
#define CAN_ID_CONTROL_CMD      0x100U
#define CAN_ID_SENSOR_DISTANCE  0x200U
#define CAN_ID_TELEMETRY_STATUS 0x300U

/* --- AEB Thresholds (Cấu hình khoảng cách an toàn cm) --- */
#define AEB_WARNING_DIST_CM     50U
#define AEB_BRAKING_DIST_CM     20U
#define AEB_WARNING_SPEED       50U

/**
 * @brief Khởi tạo module CAN App cho ECU2
 */
void ECU2_CAN_Init(void);

/**
 * @brief Hàm kiểm tra và xử lý bản tin CAN đến (Polling mode).
 *        Cần gọi liên tục trong vòng lặp while(1) của ECU2.
 */
void ECU2_CAN_ProcessRx(void);

/**
 * @brief Đóng gói dữ liệu Telemetry (Speed, AEB state, v.v.) và gửi lên CAN bus.
 *        Cần gọi định kỳ (ví dụ 100ms) bằng SysTick hoặc Timer.
 */
void ECU2_CAN_SendTelemetry(void);

#ifdef __cplusplus
}
#endif

#endif /* ECU2_CAN_APP_H */
