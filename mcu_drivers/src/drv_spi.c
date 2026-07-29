/*
 * drv_spi.c
 *
 *  Created on: 23 Jul 2026
 *      Author: trong
 */

#include "drv_spi.h"

void SPI2_Init(void) {
    // 1. Bật clock cho GPIOB và SPI2
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    // 2. Cấu hình chân GPIOB (PB12: Out GP PP, PB13: AF PP, PB14: Input, PB15: AF PP)
    GPIOB->CRH &= ~(0xFFFFFFFFUL); // Clear cấu hình từ PB8-PB15
    GPIOB->CRH |= (0x03 << 16);    // PB12: Output 50MHz, Push-Pull (CSN)
    GPIOB->CRH |= (0x0B << 20);    // PB13: AF Output 50MHz, Push-Pull (SCK)
    GPIOB->CRH |= (0x04 << 24);    // PB14: Input Floating (MISO)
    GPIOB->CRH |= (0x0B << 28);    // PB15: AF Output 50MHz, Push-Pull (MOSI)

    SPI2_CSN_Deselect();

    // 3. Cấu hình SPI2 thanh ghi CR1
    // Master Mode, Baudrate = PCLK1 / 16, SSM = 1, SSI = 1
    SPI2->CR1 = 0;
    SPI2->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM;
    SPI2->CR1 |= (0x03 << 3); // Prescaler /16
    SPI2->CR1 |= SPI_CR1_SPE;  // Bật bộ SPI2
}

uint8_t SPI2_TransmitReceive(uint8_t data) {
    while (!(SPI2->SR & SPI_SR_TXE)); // Chờ bộ đệm phát rảnh
    SPI2->DR = data;
    while (!(SPI2->SR & SPI_SR_RXNE)); // Chờ nhận xong
    return (uint8_t)(SPI2->DR);
}

void SPI2_CSN_Select(void) {
    SPI2_CSN_PORT->BRR = (1 << SPI2_CSN_PIN); // Kéo xuống LOW
}

void SPI2_CSN_Deselect(void) {
    SPI2_CSN_PORT->BSRR = (1 << SPI2_CSN_PIN); // Kéo lên HIGH
}
