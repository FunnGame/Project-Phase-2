/*
 * rf_protocol.c
 */

#include "rf_protocol.h"
#include "nrf24_hal.h"
#include <string.h>

static RF_ControlPacket_t currentPacket;
static RF_Statistics_t    stats;
static uint32_t           lastRxTime_ms = 0;
static uint8_t            lastSequence  = 0;
static bool               isConnected   = false;
static bool               seqInitialized = false;

/*
 * Thuật toán CRC-8 (Polynomial 0x07)
 */
static uint8_t RF_CalculateCRC8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0x00;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
        }
    }
    return crc;
}

static bool RF_ValidatePacket(const RF_ControlPacket_t *packet) {
    /* Tính CRC trên tất cả các byte ngoại trừ byte CRC cuối cùng */
    uint8_t calc_crc = RF_CalculateCRC8((const uint8_t *)packet, sizeof(RF_ControlPacket_t) - 1);

    if (calc_crc != packet->crc) {
        stats.crc_errors++;
        return false;
    }
    return true;
}

static bool RF_ProcessSequence(uint8_t new_seq) {
    if (!seqInitialized) {
        lastSequence = new_seq;
        seqInitialized = true;
        stats.rx_valid++;
        return true;
    }

    uint8_t gap = new_seq - lastSequence;

    if (gap == 0) {
        stats.rx_duplicates++;
        return false; /* Bỏ qua gói tin lặp */
    }

    if (gap > 1) {
        /* Đếm số lượng gói bị rớt giữa không trung */
        stats.rx_dropped += (gap - 1);
    }

    lastSequence = new_seq;
    stats.rx_valid++;
    return true;
}

void RF_Init(void) {
    memset(&currentPacket, 0, sizeof(currentPacket));
    memset(&stats, 0, sizeof(stats));
    isConnected = false;
    seqInitialized = false;
    lastRxTime_ms = 0;

    currentPacket.brake = 100;
}

void RF_Update(uint32_t sys_ticks_ms) {
    /* 1. KIỂM TRA FAILSAFE WATCHDOG */
    if (isConnected && ((sys_ticks_ms - lastRxTime_ms) > RF_TIMEOUT_MS)) {
        isConnected = false;

        /* Ghi đè lệnh điều khiển khẩn cấp: Cắt ga, nhả lái, đạp phanh 100% */
        currentPacket.throttle = 0;
        currentPacket.steering = 0;
        currentPacket.brake    = 100;
    }

    /* 2. NHẬN VÀ XỬ LÝ DỮ LIỆU RF */
    if (!NRF24_Available()) {
        return;
    }

    RF_ControlPacket_t tempPacket;
    NRF24_Receive(&tempPacket);

    if (!RF_ValidatePacket(&tempPacket)) {
        return; /* Hỏng CRC, vứt bỏ gói */
    }

    if (!RF_ProcessSequence(tempPacket.sequence)) {
        return; /* Gói bị lặp lại, không áp dụng */
    }

    /* Cập nhật dữ liệu hợp lệ */
    currentPacket = tempPacket;
    lastRxTime_ms = sys_ticks_ms;
    isConnected   = true;
}

bool RF_IsConnected(void) {
    return isConnected;
}

RF_ControlPacket_t RF_GetControl(void) {
    return currentPacket;
}

bool RF_QueueTelemetry(const RF_TelemetryPacket_t *telem) {
    /* Đẩy dữ liệu Telemetry xuống module nRF24 để đợi sẵn ở bộ đệm ACK */
    /* Giả định hàm NRF24_WriteAckPayload hỗ trợ truyền ở pipe 0 */
    return NRF24_WriteAckPayload(0, (const uint8_t *)telem, sizeof(RF_TelemetryPacket_t));
}

RF_Statistics_t RF_GetStats(void) {
    return stats;
}
