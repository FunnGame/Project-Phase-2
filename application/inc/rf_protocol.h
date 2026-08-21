/*
 * rf_protocol.h
 *
 * Tối ưu hóa: Bổ sung Auto-ACK Telemetry, Failsafe Watchdog & Link Statistics.
 */

#ifndef RF_PROTOCOL_H
#define RF_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define RF_TIMEOUT_MS 500U  /* Ngưỡng mất sóng an toàn */

/* 1. Downlink: Lệnh điều khiển từ trạm (Chuẩn 6 bytes data + 1 byte CRC) */
typedef struct __attribute__((packed)) {
    int8_t  steering;
    uint8_t throttle;
    uint8_t brake;
    uint8_t buttons;
    uint8_t sequence;
    uint8_t crc;
} RF_ControlPacket_t;

/* 2. Uplink: Dữ liệu gửi ngược lên trạm qua Auto-ACK (Chuẩn 14 bytes) */
typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  seq_echo;
    uint8_t  state;
    uint8_t  faults;
    uint8_t  throttle;
    uint8_t  brake;
    int16_t  speed;
    uint16_t range;
    uint8_t  health;
    uint8_t  slow_id;
    uint8_t  slow_val;
    uint8_t  crc;
} RF_TelemetryPacket_t;

/* 3. Thống kê chất lượng sóng (Phục vụ debug và báo cáo Telemetry) */
typedef struct {
    uint32_t rx_valid;
    uint32_t rx_dropped;
    uint32_t rx_duplicates;
    uint32_t crc_errors;
} RF_Statistics_t;

/* --- Core API --- */
void RF_Init(void);

/* Cập nhật trạng thái RF. Phải gọi liên tục trong main loop.
 * @param sys_ticks_ms: Thời gian hệ thống hiện tại (ms) để tính Failsafe */
void RF_Update(uint32_t sys_ticks_ms);

bool RF_IsConnected(void);
RF_ControlPacket_t RF_GetControl(void);

/* Nạp dữ liệu Telemetry vào phần cứng nRF24 để tự động gửi kèm ACK */
bool RF_QueueTelemetry(const RF_TelemetryPacket_t *telem);

/* Lấy thống kê chất lượng mạng */
RF_Statistics_t RF_GetStats(void);

#endif /* RF_PROTOCOL_H */
