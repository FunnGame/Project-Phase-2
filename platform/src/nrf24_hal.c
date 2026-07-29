/*
 * nrf24_hal.c
 *
 *  Created on: 23 Jul 2026
 *      Author: trong
 */

#include "nrf24_hal.h"
#include "drv_spi.h"

// Chân CE kết nối với PB11 (GPIO Output)
#define NRF24_CE_LOW()   (GPIOB->BRR = (1 << 11))
#define NRF24_CE_HIGH()  (GPIOB->BSRR = (1 << 11))

void NRF24_WriteReg(uint8_t reg, uint8_t value) {
    SPI2_CSN_Select();
    SPI2_TransmitReceive(0x20 | reg); // Lệnh Write Register
    SPI2_TransmitReceive(value);
    SPI2_CSN_Deselect();
}

void NRF24_Init(void) {
    SPI2_Init();

    // Cấu hình chân PB11 làm CE Output
    GPIOB->CRH &= ~(0x0F << 12);
    GPIOB->CRH |= (0x03 << 12);

    NRF24_CE_LOW();

    NRF24_WriteReg(0x00, 0x0F); // CONFIG: Bật CRC, PWR_UP, PRIM_RX (Chế độ nhận)
    NRF24_WriteReg(0x01, 0x00); // Disable Auto ACK (để test đơn giản)
    NRF24_WriteReg(0x05, 76);   // RF Channel 76
    NRF24_WriteReg(0x11, sizeof(RF_ControlPacket_t)); // Payload size

    NRF24_CE_HIGH(); // Bật lắng nghe sóng
}

uint8_t NRF24_Available(void) {
    SPI2_CSN_Select();
    uint8_t status = SPI2_TransmitReceive(0xFF); // Lấy thanh ghi STATUS
    SPI2_CSN_Deselect();
    return (status & (1 << 6)); // Trả về 1 nếu có dữ liệu RX_DR
}

void NRF24_Receive(RF_ControlPacket_t *packet) {
    uint8_t *ptr = (uint8_t*)packet;
    SPI2_CSN_Select();
    SPI2_TransmitReceive(0x61); // Lệnh R_RX_PAYLOAD
    for (uint8_t i = 0; i < sizeof(RF_ControlPacket_t); i++) {
        ptr[i] = SPI2_TransmitReceive(0xFF);
    }
    SPI2_CSN_Deselect();

    // Xóa cờ RX_DR
    NRF24_WriteReg(0x07, (1 << 6));
}
