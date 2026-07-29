/*
 * drv_spi.h
 *
 *  Created on: 23 Jul 2026
 *      Author: trong
 */

#ifndef INC_DRV_SPI_H_
#define INC_DRV_SPI_H_

#include "stm32f1xx.h"
#include <stdint.h>

// Định nghĩa các chân SPI2
#define SPI2_CSN_PORT    GPIOB
#define SPI2_CSN_PIN     12   // Chân CSN chọn chip

void SPI2_Init(void);
uint8_t SPI2_TransmitReceive(uint8_t data);
void SPI2_CSN_Select(void);
void SPI2_CSN_Deselect(void);

#endif /* INC_DRV_SPI_H_ */
