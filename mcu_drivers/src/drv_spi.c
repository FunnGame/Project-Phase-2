/*
 * drv_spi.c
 * Author: trong
 */
#include "drv_spi.h"

void SPI2_Init(void) {
    /* 1. Bật clock cho GPIOB và SPI2 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    /* 2. Cấu hình chân GPIOB */
    /* Clear cấu hình từ PB8-PB15 */
    GPIOB->CRH &= ~(0xFFFFFFFFUL);

    /* PB11: Output 50MHz, Push-Pull (CE) */
    GPIOB->CRH |= (0x03UL << 12);
    /* PB12: Output 50MHz, Push-Pull (CSN) */
    GPIOB->CRH |= (0x03UL << 16);
    /* PB13: AF Output 50MHz, Push-Pull (SCK) */
    GPIOB->CRH |= (0x0BUL << 20);
    /* PB14: Input Floating (MISO) */
    GPIOB->CRH |= (0x04UL << 24);
    /* PB15: AF Output 50MHz, Push-Pull (MOSI) */
    GPIOB->CRH |= (0x0BUL << 28);

    /* Đặt trạng thái mặc định: CSN cao (không chọn chip), CE thấp (không thu/phát) */
    SPI2_CSN_Write(true);
    SPI2_CE_Write(false);

    /* 3. Cấu hình SPI2 thanh ghi CR1 */
    /* Master Mode, Baudrate = PCLK1 / 16, SSM = 1, SSI = 1 */
    SPI2->CR1 = 0;
    SPI2->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM;
    SPI2->CR1 |= (0x03U << 3); /* Prescaler /16 */
    SPI2->CR1 |= SPI_CR1_SPE;  /* Bật bộ SPI2 */
}

uint8_t SPI2_TransmitReceive(uint8_t data) {
    while (!(SPI2->SR & SPI_SR_TXE));
    SPI2->DR = data;
    while (!(SPI2->SR & SPI_SR_RXNE));
    return (uint8_t)(SPI2->DR);
}

void SPI2_CSN_Write(bool high) {
    if (high) {
        SPI2_CSN_PORT->BSRR = (1U << SPI2_CSN_PIN);
    } else {
        SPI2_CSN_PORT->BRR = (1U << SPI2_CSN_PIN);
    }
}

void SPI2_CE_Write(bool high) {
    if (high) {
        SPI2_CE_PORT->BSRR = (1U << SPI2_CE_PIN);
    } else {
        SPI2_CE_PORT->BRR = (1U << SPI2_CE_PIN);
    }
}

void SPI2_DelayUs(uint32_t us) {
    /* Hàm delay tương đối dựa trên vòng lặp NOP.
     * Hệ số này phù hợp với Clock 72MHz. Cần tinh chỉnh nếu Clock thay đổi. */
    uint32_t loops = us * (SystemCoreClock / 1000000U / 5U);
    for (uint32_t i = 0; i < loops; ++i) {
        __asm__ volatile ("NOP");
    }
}
