/*
 * main.c
 * Author: trong
 */
#include "main.h"
#include "gateway.h"
#include "drv_can.h"
#include "drv_spi.h"
#include "shared/nrf24/nrf24.h"
#include "contracts/rf_protocol.h"

/* Không còn dùng rf_parser nữa, chỉ cần struct thiết bị */
static nrf24_t nrf_device;

void SysTick_Handler(void) {
    Gateway_Tick1ms();
}

int main(void) {
    /* 1. System Tick Init 1ms */
    SysTick_Config(SystemCoreClock / 1000U);

    /* 2. Hardware & Gateway Layer Init */
    Gateway_Init();
    SPI2_Init();

    /* Cấu hình HAL cho nRF24 */
    nrf24_hal_t nrf_hal = {
        .spi_transfer = SPI2_TransmitReceive,
        .csn_write    = SPI2_CSN_Write,
        .ce_write     = SPI2_CE_Write,
        .delay_us     = SPI2_DelayUs
    };

    /* Cấu hình thông số nRF24 theo chuẩn chung của Vinh */
    nrf24_config_t nrf_config = {
        .channel       = 76U,
        .data_rate     = NRF24_DR_1MBPS,
        .power         = NRF24_PWR_0DBM,
        .payload_width = RF_CONTROL_FRAME_SIZE, /* Bắt buộc là 7 byte */
        .address       = {0x34, 0x43, 0x10, 0x10, 0x01},
        .auto_ack      = false
    };

    /* Khởi tạo nRF24 với 3 tham số */
    if (!nrf24_init(&nrf_device, &nrf_hal, &nrf_config)) {
        Error_Handler();
    }

    /* Đưa NRF24 vào chế độ nhận sóng ngay lập tức */
    nrf24_set_rx_mode(&nrf_device);

    /* 3. Main Operational Loop */
    while (1) {
        /* A. Luồng xử lý RF Receiver */
        if (nrf24_data_available(&nrf_device)) {
            rf_control_frame_t received_frame;

            /* Đọc nguyên một frame RF 7 byte */
            if (nrf24_read(&nrf_device, (uint8_t *)&received_frame, RF_CONTROL_FRAME_SIZE)) {
                /* Kiểm tra tính hợp lệ của frame (Magic byte, CRC,...) */
                if (rf_control_frame_valid(&received_frame)) {
                    /* Chuyển thẳng frame hợp lệ xuống Gateway */
                    Gateway_ProcessRFFrame(&received_frame);
                }
            }
        }

        /* B. Luồng xử lý nhận tin nhắn CAN Telemetry */
        CAN_RxHeader_t rx_header;
        if (DRV_CAN_Receive(&rx_header)) {
            if (rx_header.std_id == CAN_ID_TELEMETRY_STATUS) {
                CAN_TelemetryStatus_t telemetry;
                CANMatrix_UnpackTelemetry(rx_header.data, &telemetry);
                /* Sẵn sàng cho luồng đẩy Telemetry ngược về HMI trạm */
            }
        }
    }
}

void Error_Handler(void) {
    while (1) {
    }
}
