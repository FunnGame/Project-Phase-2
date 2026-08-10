/*
 * drv_spi.h
 * Author: trong
 */
#ifndef INC_DRV_SPI_H_
#define INC_DRV_SPI_H_

#include "stm32f1xx.h"
#include <stdint.h>
#include <stdbool.h>

/* Định nghĩa các chân điều khiển nRF24 (SPI2) */
#define SPI2_CSN_PORT    GPIOB
#define SPI2_CSN_PIN     12   // Chân CSN (Chip Select Not)

#define SPI2_CE_PORT     GPIOB
#define SPI2_CE_PIN      11   // Chân CE (Chip Enable)

void SPI2_Init(void);
uint8_t SPI2_TransmitReceive(uint8_t data);

/* Các hàm cung cấp cho nRF24 HAL */
void SPI2_CSN_Write(bool high);
void SPI2_CE_Write(bool high);
void SPI2_DelayUs(uint32_t us);

#endif /* INC_DRV_SPI_H_ */
