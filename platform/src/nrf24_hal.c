/*
 * nrf24_hal.c
 *
 *
 * NRF24L01 Hardware Abstraction Layer
 * Communication ECU (Receiver)
 */

#include "nrf24_hal.h"
#include "drv_spi.h"

/**
 * @brief Hàm nội bộ (Private wrapper) để chuyển đổi từ chuẩn block transfer
 *        của Generic Driver sang single-byte transfer của ECU3.
 *
 * @param tx  Con trỏ tới mảng dữ liệu cần gửi. Nếu NULL, sẽ gửi mã NOP (0xFF).
 * @param rx  Con trỏ tới mảng chứa dữ liệu nhận. Nếu NULL, bỏ qua dữ liệu nhận.
 * @param len Số lượng byte cần truyền/nhận.
 */
static void NRF24_HAL_SPI_Transfer_Wrapper(const uint8_t *tx, uint8_t *rx, size_t len) {
    if (len == 0) {
        return;
    }

    for (size_t i = 0; i < len; ++i) {
        /* NRF24 yêu cầu xuất dummy byte (0xFF) để duy trì clock nếu không có data gửi */
        uint8_t tx_byte = (tx != NULL) ? tx[i] : 0xFF;

        /* Gọi xuống MCU Driver (Direct Register level) */
        uint8_t rx_byte = SPI2_TransmitReceive(tx_byte);

        /* Chỉ ghi vào buffer nhận nếu generic driver có yêu cầu lấy data */
        if (rx != NULL) {
            rx[i] = rx_byte;
        }
    }
}

/**
 * @brief Khởi tạo đối tượng struct chứa các con trỏ hàm.
 *        Được khai báo 'static const' để bảo vệ vùng nhớ trong Flash.
 */
static const nrf24_hal_t ecu3_nrf24_adapter = {
    .spi_transfer = NRF24_HAL_SPI_Transfer_Wrapper,
    .csn_write    = SPI2_CSN_Write,
    .ce_write     = SPI2_CE_Write,
    .delay_us     = SPI2_DelayUs
};

const nrf24_hal_t* NRF24_HAL_GetAdapter(void) {
    return &ecu3_nrf24_adapter;
}
