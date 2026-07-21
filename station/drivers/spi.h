/**
 ******************************************************************************
 * @file    spi.h
 * @brief   SPI master driver for the STM32F446RE.
 *
 * Full-duplex, blocking SPI master intended for the control station's SPI
 * peripherals: the ILI9341 TFT and the nRF24L01+ radio. Chip-select is left
 * to the caller (software NSS via a GPIO) so several devices can share one bus
 * with independent CS lines.
 *
 * The GPIO pins (SCK/MISO/MOSI) must be configured in the correct alternate
 * function separately (see gpio_init + the device's datasheet AF mapping).
 ******************************************************************************
 */
#ifndef STATION_DRIVERS_SPI_H
#define STATION_DRIVERS_SPI_H

#include "drivers_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Clock polarity/phase (SPI mode 0..3). */
typedef enum {
    SPI_MODE0 = 0,  /**< CPOL=0, CPHA=0 (ILI9341, nRF24 default).          */
    SPI_MODE1,      /**< CPOL=0, CPHA=1.                                   */
    SPI_MODE2,      /**< CPOL=1, CPHA=0.                                   */
    SPI_MODE3,      /**< CPOL=1, CPHA=1.                                   */
} spi_mode_t;

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
} spi_baud_t;

/** @brief Bit transmission order. */
typedef enum {
    SPI_BIT_MSB_FIRST = 0,
    SPI_BIT_LSB_FIRST,
} spi_bit_order_t;

/** @brief Frame data size. */
typedef enum {
    SPI_DATASIZE_8BIT = 0,
    SPI_DATASIZE_16BIT,
} spi_datasize_t;

/** @brief SPI master configuration. */
typedef struct {
    spi_mode_t      mode;       /**< Clock polarity/phase.                 */
    spi_baud_t      baud;       /**< Clock prescaler off PCLK.             */
    spi_bit_order_t bit_order;  /**< MSB or LSB first.                     */
    spi_datasize_t  datasize;   /**< 8- or 16-bit frames.                  */
} spi_config_t;

/* -------------------------------------------------------------------------- */
/*  Configuration                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Enable @p spi's clock and configure it as a master (software NSS).
 * @param spi One of SPI1..SPI4.
 * @param cfg Configuration (must not be NULL).
 * @return DRV_OK or DRV_INVALID_PARAM.
 */
drv_status_t spi_init(SPI_TypeDef *spi, const spi_config_t *cfg);

/** @brief Enable the peripheral (SPE). Called for you by spi_init(). */
void spi_enable(SPI_TypeDef *spi);
/** @brief Disable the peripheral after the bus is idle. */
void spi_disable(SPI_TypeDef *spi);

/* -------------------------------------------------------------------------- */
/*  Blocking transfers                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Exchange a single byte (write @p tx, return the received byte).
 * @note  8-bit data size only.
 */
uint8_t spi_transfer_byte(SPI_TypeDef *spi, uint8_t tx);

/**
 * @brief Full-duplex transfer of @p len bytes.
 * @param spi SPI instance.
 * @param tx  Bytes to send, or NULL to clock out 0x00.
 * @param rx  Buffer for received bytes, or NULL to discard.
 * @param len Number of bytes.
 * @return DRV_OK, DRV_INVALID_PARAM, or DRV_TIMEOUT.
 */
drv_status_t spi_transfer(SPI_TypeDef *spi, const uint8_t *tx, uint8_t *rx,
                          size_t len);

/** @brief Transmit @p len bytes, discarding the received data. */
drv_status_t spi_write(SPI_TypeDef *spi, const uint8_t *tx, size_t len);

/** @brief Receive @p len bytes, clocking out 0x00. */
drv_status_t spi_read(SPI_TypeDef *spi, uint8_t *rx, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* STATION_DRIVERS_SPI_H */
