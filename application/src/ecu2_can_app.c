/*
 * ecu2_can_app.c
 *
 * Description: Application layer for handling CAN Rx/Tx on ECU2 (Motor Control).
 */

#include "ecu2_can_app.h"
#include "drv_can.h"
#include "control_app.h"     /* Cung cấp ControlApp_Drive, ControlApp_Stop */
#include "aeb.h"             /* Cung cấp AEB_Execute, AEB_State_t */
#include "motor_encoder.h"   /* Cung cấp MotorEncoder_GetLeftRPM, GetRightRPM */

/* Lưu trữ trạng thái hệ thống nội bộ của ECU2 */
static uint16_t g_current_distance_mm = 0U;
static AEB_State_t g_current_aeb_state = AEB_STATE_IDLE;
static uint8_t g_battery_percent = 100U; /* Giả lập % pin */

void ECU2_CAN_Init(void) {
    /* Khởi tạo phần cứng CAN dựa trên driver của ECU3 */
    DRV_CAN_Init();
}

void ECU2_CAN_ProcessRx(void) {
    CAN_RxHeader_t rx_msg;

    /* Đọc bản tin CAN (nếu có) từ FIFO */
    if (DRV_CAN_Receive(&rx_msg)) {

        switch (rx_msg.std_id) {
            case CAN_ID_CONTROL_CMD:
            {
                /* 1. Xử lý bản tin 0x100: Lệnh điều khiển từ Gateway ECU3 */
                int8_t throttle = (int8_t)rx_msg.data[0];
                int8_t steering = (int8_t)rx_msg.data[1];
                uint8_t brake   = rx_msg.data[2];
                /* byte[3]: buttons, byte[4]: seq, byte[7]: crc - hiện chưa dùng tới logic ngắt ở ECU2 */

                /* Kiểm tra Failsafe/Safe Stop từ ECU3 (mất RF sẽ ép brake = 100) hoặc AEB đang can thiệp */
                if (brake > 0U || g_current_aeb_state == AEB_STATE_BRAKING) {
                    ControlApp_Stop();
                } else {
                    ControlApp_Drive(throttle, steering);
                }
                break;
            }

            case CAN_ID_SENSOR_DISTANCE:
            {
                /* 2. Xử lý bản tin 0x200: Khoảng cách từ Sensor ECU1 */
                g_current_distance_mm = (uint16_t)rx_msg.data[0] | ((uint16_t)rx_msg.data[1] << 8);

                /* Đổi mm sang cm để tương thích với hàm AEB của ECU2 */
                float distance_cm = (float)g_current_distance_mm / 10.0f;

                /* Chạy thuật toán Phanh khẩn cấp tự động */
                g_current_aeb_state = AEB_Execute(distance_cm,
                                                  AEB_WARNING_DIST_CM,
                                                  AEB_BRAKING_DIST_CM,
                                                  AEB_WARNING_SPEED);

                /* Nếu khoảng cách vi phạm vạch đỏ, cưỡng chế ngắt động cơ */
                if (g_current_aeb_state == AEB_STATE_BRAKING) {
                    ControlApp_Stop();
                }
                break;
            }

            default:
                /* Bỏ qua các ID không hợp lệ trong ma trận */
                break;
        }
    }
}

void ECU2_CAN_SendTelemetry(void) {
    CAN_TxHeader_t tx_msg;

    /* 3. Đóng gói bản tin 0x300: Gửi Telemetry ngược về ECU3 */
    tx_msg.std_id = CAN_ID_TELEMETRY_STATUS;
    tx_msg.dlc = 8U;

    /* Lấy tốc độ thực tế từ module Encoder của ECU2 */
    float left_rpm  = MotorEncoder_GetLeftRPM();
    float right_rpm = MotorEncoder_GetRightRPM();
    uint16_t avg_rpm = (uint16_t)((left_rpm + right_rpm) / 2.0f);

    /* Đóng gói Data theo chuẩn Little-Endian */
    tx_msg.data[0] = (uint8_t)(avg_rpm & 0xFFU);          /* speed_rpm (Low Byte) */
    tx_msg.data[1] = (uint8_t)((avg_rpm >> 8) & 0xFFU);   /* speed_rpm (High Byte) */
    tx_msg.data[2] = 0U;                                  /* acc_active (Chưa có logic State Machine) */
    tx_msg.data[3] = (g_current_aeb_state != AEB_STATE_IDLE) ? 1U : 0U; /* aeb_active */
    tx_msg.data[4] = g_battery_percent;                   /* battery_percent */
    tx_msg.data[5] = 0U;                                  /* reserved */
    tx_msg.data[6] = 0U;                                  /* reserved */
    tx_msg.data[7] = 0U;                                  /* reserved (Có thể cập nhật CRC8 ở đây) */

    DRV_CAN_Transmit(&tx_msg);
}
