/**
 ******************************************************************************
 * @file    drv_spi.h
 * @brief   SPI master driver for the STM32F103.
 *
 * Works with either SPI instance, configured through a struct rather than
 * hardcoded registers and pins. Chip-select is deliberately NOT owned by this
 * driver: pass the CS pin per transaction, so several devices (nRF24, SD card,
 * ...) can share one bus with independent CS lines.
 *
 * Default F103 pin mapping (no AFIO remap):
 *   SPI1 - PA5 SCK, PA6 MISO, PA7 MOSI
 *   SPI2 - PB13 SCK, PB14 MISO, PB15 MOSI
 ******************************************************************************
 */
#ifndef DRV_SPI_H_
#define DRV_SPI_H_

#include "drv_common.h"
#include "drv_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Clock polarity / phase. */
typedef enum {
    SPI_MODE0 = 0,  /**< CPOL=0, CPHA=0 — nRF24, most devices.            */
    SPI_MODE1,      /**< CPOL=0, CPHA=1.                                  */
    SPI_MODE2,      /**< CPOL=1, CPHA=0.                                  */
    SPI_MODE3,      /**< CPOL=1, CPHA=1.                                  */
} SPI_Mode;

/** @brief Baud rate prescaler: f_SPI = PCLK / 2^(value+1). */
typedef enum {
    SPI_BAUD_DIV2 = 0,
    SPI_BAUD_DIV4,
    SPI_BAUD_DIV8,
    SPI_BAUD_DIV16,
    SPI_BAUD_DIV32,
    SPI_BAUD_DIV64,
    SPI_BAUD_DIV128,
    SPI_BAUD_DIV256,
} SPI_Baud;

/** @brief Bit order. */
typedef enum {
    SPI_MSB_FIRST = 0,
    SPI_LSB_FIRST,
} SPI_BitOrder;

/** @brief SPI master configuration. */
typedef struct {
    SPI_TypeDef  *spi;        /**< SPI1 or SPI2.                          */
    GPIO_TypeDef *port;       /**< Port carrying SCK/MISO/MOSI.           */
    uint8_t       sck_pin;
    uint8_t       miso_pin;
    uint8_t       mosi_pin;
    SPI_Mode      mode;
    SPI_Baud      baud;
    SPI_BitOrder  bit_order;
} SPI_Config;

/**
 * @brief Enable the clocks, configure the three bus pins and start @p spi as a
 *        master with software slave management.
 * @return DRV_OK or DRV_INVALID_PARAM.
 *
 * Only the nibbles of the three bus pins are modified, so neighbouring pins on
 * the same port keep their configuration.
 */
DRV_Status DRV_SPI_Init(const SPI_Config *cfg);

/** @brief Exchange one byte: writes @p data, returns what was clocked back. */
uint8_t DRV_SPI_Transfer(SPI_TypeDef *spi, uint8_t data);

/**
 * @brief Full-duplex transfer of @p len bytes.
 * @param tx Bytes to send, or NULL to clock out zeros.
 * @param rx Buffer for received bytes, or NULL to discard.
 * @return DRV_OK, DRV_INVALID_PARAM, or DRV_TIMEOUT.
 */
DRV_Status DRV_SPI_TransferBuffer(SPI_TypeDef *spi, const uint8_t *tx,
                                  uint8_t *rx, size_t len);

/* -------------------------------------------------------------------------- */
/*  Chip select (caller-owned, so one bus can serve several devices)          */
/* -------------------------------------------------------------------------- */

/** @brief Configure @p pin as a chip-select output, idle (high). */
DRV_Status DRV_SPI_InitCS(GPIO_TypeDef *port, uint8_t pin);

/** @brief Assert chip select (drive low). */
static inline void DRV_SPI_Select(GPIO_TypeDef *port, uint8_t pin)
{
    DRV_GPIO_Clear(port, pin);
}

/** @brief Release chip select (drive high). */
static inline void DRV_SPI_Deselect(GPIO_TypeDef *port, uint8_t pin)
{
    DRV_GPIO_Set(port, pin);
}

#ifdef __cplusplus
}
#endif

#endif /* DRV_SPI_H_ */
