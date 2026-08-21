/*
 * gateway.c
 *
 */

#include "gateway.h"
#include "can_matrix.h"
#include "drv_can.h"
#include "nrf24.h"
#include <stddef.h>
#include <string.h>

#define RF_TIMEOUT_MS        500U   /* Ngưỡng mất RF kích hoạt Safe Stop */
#define CAN_CONTROL_CYCLE_MS 20U    /* Chu kỳ 50Hz gửi lệnh điều khiển */

extern nrf24_t s_radio;

static ECU3_State_t          current_state = ECU3_STATE_INIT;
static uint32_t              rf_last_rx_timestamp = 0U;
static uint32_t              can_tx_timer = 0U;
static uint32_t              system_ticks = 0U;
static rf_control_frame_t    latest_rf_frame;
static uint8_t               can_sequence = 0U;

/* ---- Biến lưu trữ dữ liệu Uplink (Telemetry) ---- */
static CAN_TelemetryStatus_t current_telemetry = {0};
static uint16_t              current_sensor_distance = RF_TELEM_RANGE_NONE;
static uint8_t               slow_id_counter = 0;

static void Gateway_SendSafeStopCANFrame(void) {
    CAN_ControlCmd_t safe_cmd = {
        .throttle = 0,
        .steering = 0,
        .brake    = 100, /* Đạp phanh khẩn cấp để dừng xe */
        .buttons  = 0,
        .seq      = can_sequence++
    };

    CAN_TxHeader_t tx;
    tx.std_id = CAN_ID_CONTROL_CMD;
    tx.dlc    = 8;
    CANMatrix_PackControlCmd(&safe_cmd, tx.data);
    DRV_CAN_Transmit(&tx);
}

/*
 * Nạp dữ liệu vào ACK Payload.
 * Hàm này được gọi ngay sau khi nhận thành công 1 frame RF.
 */
static void Gateway_QueueTelemetryACK(uint8_t seq_echo) {
    rf_telemetry_frame_t telem;
    memset(&telem, 0, sizeof(telem));

    telem.magic    = RF_TELEM_MAGIC;
    telem.seq_echo = seq_echo;

    /* Lấy trạng thái từ nhánh ECU2 */
    telem.state    = current_telemetry.state;
    telem.faults   = current_telemetry.faults;
    telem.speed    = current_telemetry.speed;
    telem.throttle = current_telemetry.throttle;
    telem.brake    = current_telemetry.brake;

    /* Lấy khoảng cách từ nhánh Sensor_ECU */
    telem.range    = current_sensor_distance;

    /* Cờ trạng thái các Node */
    telem.health   = RF_TELEM_NODE_GW | RF_TELEM_NODE_VC | RF_TELEM_NODE_SF;
    telem.slow_id  = slow_id_counter;
    telem.slow_val = 0;

    slow_id_counter = (slow_id_counter + 1) % RF_SLOW_COUNT;

    /* Tính lại CRC-8 cho gói Telemetry */
    rf_telemetry_frame_finalize(&telem);

    /* Nạp vào FIFO của nRF24. Gói này sẽ tự động bay lên trạm vào lần giao tiếp RF tiếp theo */
    nrf24_write_ack_payload(&s_radio, 0, (const uint8_t *)&telem, sizeof(telem));
}

void Gateway_Init(void) {
    DRV_CAN_Init();
    current_state = ECU3_STATE_SAFE_STOP;
    rf_last_rx_timestamp = system_ticks;
}

void Gateway_ProcessRFFrame(const rf_control_frame_t *rf_frame) {
    if (rf_frame == NULL) return;

    latest_rf_frame = *rf_frame;
    rf_last_rx_timestamp = system_ticks;

    if (current_state == ECU3_STATE_SAFE_STOP) {
        current_state = ECU3_STATE_OPERATIONAL;
    }

    /* Quan Trọng: Nạp ngay gói Uplink vào nRF24 khi vừa nhận xong Downlink */
    Gateway_QueueTelemetryACK(rf_frame->seq);
}

/* Hàm hứng dữ liệu bất đồng bộ từ các node khác trên CAN Bus */
void Gateway_ProcessCANFrame(uint32_t can_id, const uint8_t *payload) {
    if (payload == NULL) return;

    switch (can_id) {
        case CAN_ID_TELEMETRY_STATUS:
            /* Cập nhật trạng thái từ xe (ECU2) */
            CANMatrix_UnpackTelemetry(payload, &current_telemetry);
            break;

        case CAN_ID_SENSOR_DISTANCE:
            /*
             * Cập nhật khoảng cách vật cản (ECU1).
             */
            current_sensor_distance = (uint16_t)((uint16_t)payload[0] | ((uint16_t)payload[1] << 8));
            break;

        default:
            break;
    }
}

void Gateway_Tick1ms(void) {
    system_ticks++;
    can_tx_timer++;

    /* 1. Kiểm tra Timeout RF (Safe Stop Watchdog) */
    if ((system_ticks - rf_last_rx_timestamp) > RF_TIMEOUT_MS) {
        current_state = ECU3_STATE_SAFE_STOP;
    }

    /* 2. Chu kỳ phát bản tin CAN ID 0x100 (20ms / 50Hz) */
    if (can_tx_timer >= CAN_CONTROL_CYCLE_MS) {
        can_tx_timer = 0U;

        if (current_state == ECU3_STATE_OPERATIONAL) {
            CAN_ControlCmd_t cmd = {
                .steering = latest_rf_frame.steering,
                .throttle = latest_rf_frame.throttle,
                .brake    = latest_rf_frame.brake,
                .buttons  = latest_rf_frame.buttons,
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
