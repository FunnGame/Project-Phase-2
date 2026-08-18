/**
 ******************************************************************************
 * @file    drv_uart.h
 * @brief   USART driver for the STM32F103 (blocking, register-level).
 *
 * 8N1, TX + RX. The baud rate is derived from the USART's real bus clock via
 * drv_clock (USART1 off PCLK2, USART2/3 off PCLK1), so it is correct at 8 MHz
 * HSI and at 72 MHz PLL alike.
 *
 * Pins are fixed by the peripheral:
 *   USART1  TX/RX = PA9/PA10   (or PB6/PB7 with remap)
 *   USART2  TX/RX = PA2/PA3
 *   USART3  TX/RX = PB10/PB11
 *
 * On the Blue Pill there is no on-board USB-serial bridge, so this is for a
 * USB-to-TTL adapter on the TX/RX pins (3.3 V logic; common ground).
 ******************************************************************************
 */
#ifndef DRV_UART_H_
#define DRV_UART_H_

#include "drv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief One USART configuration. */
typedef struct {
    USART_TypeDef *uart;    /**< USART1, USART2 or USART3.                 */
    uint32_t       baud;    /**< Baud rate, e.g. 115200.                   */
    bool           remap;   /**< USART1 only: true = PB6/PB7, else PA9/PA10*/
} UART_Config;

/**
 * @brief Bring up the USART described by @p cfg (clock, pins, baud, 8N1).
 * @return DRV_OK, DRV_INVALID_PARAM, or DRV_UNSUPPORTED if the bus clock
 *         cannot produce @p baud.
 */
DRV_Status DRV_UART_Init(const UART_Config *cfg);

/** @brief Send one byte (blocks until the transmit register is free). */
void DRV_UART_WriteByte(USART_TypeDef *uart, uint8_t byte);

/** @brief Send @p len bytes. */
void DRV_UART_Write(USART_TypeDef *uart, const uint8_t *data, uint16_t len);

/** @brief Send a NUL-terminated string. */
void DRV_UART_WriteString(USART_TypeDef *uart, const char *str);

/** @brief True if a received byte is waiting (RXNE set). */
bool DRV_UART_DataAvailable(USART_TypeDef *uart);

/**
 * @brief Read one byte, blocking until one arrives.
 * @note  Poll DRV_UART_DataAvailable() first if you must not block.
 */
uint8_t DRV_UART_ReadByte(USART_TypeDef *uart);

#ifdef __cplusplus
}
#endif

#endif /* DRV_UART_H_ */
