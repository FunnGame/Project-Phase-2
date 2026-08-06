/*
 * gateway.c
 *
 * Author: trong
 */


#include "gateway.h"
#include "can_matrix.h"
#include "drv_can.h"
#include <stddef.h>

#define RF_TIMEOUT_MS        500U   /* Ngưỡng mất RF kích hoạt Safe Stop */
#define CAN_CONTROL_CYCLE_MS 20U    /* Chu kỳ 50Hz gửi lệnh điều khiển */[cite: 10]

static ECU3_State_t          current_state = ECU3_STATE_INIT;
static uint32_t              rf_last_rx_timestamp = 0U;
static uint32_t              can_tx_timer = 0U;
static uint32_t              system_ticks = 0U;
static rf_control_frame_t    latest_rf_frame;
static uint8_t               can_sequence = 0U;

static void Gateway_SendSafeStopCANFrame(void) {
    CAN_ControlCmd_t safe_cmd = {
        .throttle = 0,
        .steering = 0,
        .mode     = 0, /* MANUAL */
        .flags    = 0, /* Disarmed */
        .seq      = can_sequence++
    };

    CAN_TxHeader_t tx;
    tx.std_id = CAN_ID_CONTROL_CMD;
    tx.dlc    = 8;
    CANMatrix_PackControlCmd(&safe_cmd, tx.data);
    DRV_CAN_Transmit(&tx);
}

void Gateway_Init(void) {
    DRV_CAN_Init();
    current_state = ECU3_STATE_OPERATIONAL;
    rf_last_rx_timestamp = system_ticks;
}

void Gateway_ProcessRFFrame(const rf_control_frame_t *rf_frame) {
    if (rf_frame == NULL) return;

    /* Lưu trữ frame hợp lệ đã được kiểm tra CRC*/[cite: 9]
    latest_rf_frame = *rf_frame;
    rf_last_rx_timestamp = system_ticks;

    if (current_state == ECU3_STATE_SAFE_STOP) {
        current_state = ECU3_STATE_OPERATIONAL; /* Tự động phục hồi khi có RF trở lại */
    }
}

void Gateway_Tick1ms(void) {
    system_ticks++;
    can_tx_timer++;

    /* 1. Kiểm tra Timeout RF (Safe Stop Watchdog) */[cite: 10]
    if ((system_ticks - rf_last_rx_timestamp) > RF_TIMEOUT_MS) {
        current_state = ECU3_STATE_SAFE_STOP;
    }

    /* 2. Chu kỳ phát bản tin CAN ID 0x100 (20ms / 50Hz) */[cite: 10]
    if (can_tx_timer >= CAN_CONTROL_CYCLE_MS) {
        can_tx_timer = 0U;

        if (current_state == ECU3_STATE_OPERATIONAL) {
            /* Chuyển đổi dữ liệu RF thành CAN Command */
            int16_t mapped_throttle = (int16_t)latest_rf_frame.throttle;
            if (rf_control_is_reverse(&latest_rf_frame)) {
                mapped_throttle = -mapped_throttle;
            }

            CAN_ControlCmd_t cmd = {
                .throttle = (int8_t)mapped_throttle,
                .steering = latest_rf_frame.steering,
                .mode     = 0, /* Manual mode */
                .flags    = rf_control_is_armed(&latest_rf_frame) ? 0x01U : 0x00U,
                .seq      = can_sequence++
            };

            CAN_TxHeader_t tx;
            tx.std_id = CAN_ID_CONTROL_CMD;
            tx.dlc    = 8;
            CANMatrix_PackControlCmd(&cmd, tx.data);
            DRV_CAN_Transmit(&tx);
        }
        else if (current_state == ECU3_STATE_SAFE_STOP) {
            Gateway_SendSafeStopCANFrame();
        }
    }
}

ECU3_State_t Gateway_GetState(void) {
    return current_state;
}
