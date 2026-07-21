/**
 ******************************************************************************
 * @file    uart.h
 * @brief   USART/UART driver for the STM32F446RE.
 *
 * Blocking byte/buffer/string I/O plus an optional RX-interrupt callback,
 * primarily for the control station's debug/telemetry console (USART2 is wired
 * to the Nucleo-F446RE ST-Link virtual COM port on PA2/PA3, AF7).
 *
 * The baud rate is derived from the peripheral's actual APB clock (queried
 * from the rcc driver) so it stays correct across clock reconfigurations.
 * TX/RX GPIO pins must be set to the right alternate function separately.
 ******************************************************************************
 */
#ifndef STATION_DRIVERS_UART_H
#define STATION_DRIVERS_UART_H

#include "drivers_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef UART_PROVIDE_IRQ_HANDLERS
#define UART_PROVIDE_IRQ_HANDLERS 1
#endif

/** @brief Word length (data bits, excluding parity). */
typedef enum {
    UART_WORDLEN_8 = 0,  /**< 8 data bits (M=0).                           */
    UART_WORDLEN_9,      /**< 9 data bits (M=1), or 8 data + parity.       */
} uart_wordlen_t;

/** @brief Parity mode. */
typedef enum {
    UART_PARITY_NONE = 0,
    UART_PARITY_EVEN,
    UART_PARITY_ODD,
} uart_parity_t;

/** @brief Number of stop bits (USART_CR2_STOP encoding). */
typedef enum {
    UART_STOPBITS_1   = 0x0,
    UART_STOPBITS_0P5 = 0x1,
    UART_STOPBITS_2   = 0x2,
    UART_STOPBITS_1P5 = 0x3,
} uart_stopbits_t;

/** @brief Enabled transfer directions. */
typedef enum {
    UART_DIR_TX   = 0x1,
    UART_DIR_RX   = 0x2,
    UART_DIR_TXRX = 0x3,
} uart_direction_t;

/**
 * @brief Received-byte callback, invoked from the USART RX ISR.
 * @param usart The instance that received a byte.
 * @param byte  The received data byte.
 * @param ctx   Opaque pointer from uart_attach_rx_irq().
 */
typedef void (*uart_rx_callback_t)(USART_TypeDef *usart, uint8_t byte, void *ctx);

/** @brief UART configuration. */
typedef struct {
    uint32_t         baud;       /**< Baud rate, e.g. 115200.              */
    uart_wordlen_t   wordlen;    /**< Data word length.                    */
    uart_parity_t    parity;     /**< Parity mode.                         */
    uart_stopbits_t  stopbits;   /**< Stop bits.                           */
    uart_direction_t direction;  /**< TX / RX / both.                      */
} uart_config_t;

/* -------------------------------------------------------------------------- */
/*  Configuration                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Enable @p usart's clock and configure it from @p cfg.
 * @param usart One of USART1..USART3, UART4/UART5, USART6.
 * @param cfg   Configuration (must not be NULL).
 * @return DRV_OK or DRV_INVALID_PARAM.
 */
drv_status_t uart_init(USART_TypeDef *usart, const uart_config_t *cfg);

/**
 * @brief Convenience: 8N1 configuration at @p baud, TX+RX enabled.
 */
drv_status_t uart_init_8n1(USART_TypeDef *usart, uint32_t baud);

/* -------------------------------------------------------------------------- */
/*  Blocking I/O                                                              */
/* -------------------------------------------------------------------------- */

/** @brief Send one byte (blocks until the TX register is free). */
void uart_write_byte(USART_TypeDef *usart, uint8_t byte);
/** @brief Send @p len bytes. */
void uart_write(USART_TypeDef *usart, const uint8_t *data, size_t len);
/** @brief Send a NUL-terminated string. */
void uart_write_string(USART_TypeDef *usart, const char *str);

/** @brief True if a received byte is waiting in RDR. */
bool uart_rx_ready(USART_TypeDef *usart);
/** @brief Block until a byte arrives and return it. */
uint8_t uart_read_byte(USART_TypeDef *usart);
/**
 * @brief Read up to @p len bytes, blocking for each.
 * @return Number of bytes read (always @p len for the blocking variant).
 */
size_t uart_read(USART_TypeDef *usart, uint8_t *buf, size_t len);

/* -------------------------------------------------------------------------- */
/*  RX interrupt                                                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Enable RXNE interrupts on @p usart and route them to @p cb.
 * @param cb       Callback invoked per received byte, or NULL to disable.
 * @param ctx      Opaque pointer forwarded to @p cb.
 * @param priority NVIC priority (0 = highest).
 * @return DRV_OK or DRV_INVALID_PARAM.
 */
drv_status_t uart_attach_rx_irq(USART_TypeDef *usart, uart_rx_callback_t cb,
                                void *ctx, uint32_t priority);

/** @brief Service handler; drains RXNE and calls the callback. */
void uart_irq_dispatch(USART_TypeDef *usart);

#ifdef __cplusplus
}
#endif

#endif /* STATION_DRIVERS_UART_H */
