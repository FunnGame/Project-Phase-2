/**
 ******************************************************************************
 * @file    drv_i2c.h
 * @brief   I2C master driver for the STM32F103.
 *
 * Register-level, blocking, HAL-free — the canonical I2C for the car. Bus
 * timing is derived from the real PCLK1 via drv_clock, so 100 kHz / 400 kHz are
 * correct at any SYSCLK (unlike a hardcoded "36 MHz" divider).
 *
 * The pins are fixed by the peripheral, all on GPIOB:
 *   I2C1  SCL/SDA = PB6/PB7   (or PB8/PB9 with remap)
 *   I2C2  SCL/SDA = PB10/PB11 (no remap)
 *
 * Access is register-oriented (8-bit register index): the on-chip sensors this
 * drives — VL53L0X, MPU6050 — all address registers with a single byte.
 ******************************************************************************
 */
#ifndef DRV_I2C_H_
#define DRV_I2C_H_

#include "drv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief One I2C master. */
typedef struct {
    I2C_TypeDef *i2c;        /**< I2C1 or I2C2.                             */
    uint32_t     speed_hz;   /**< 100000 (standard) or 400000 (fast).      */
    bool         remap;      /**< I2C1 only: true = PB8/PB9, false = PB6/PB7*/
} I2C_Config;

/**
 * @brief Bring up the bus described by @p cfg (clocks, pins, timing).
 * @return DRV_OK, DRV_INVALID_PARAM, or DRV_UNSUPPORTED if PCLK1 is out of the
 *         2..50 MHz range the peripheral requires.
 */
DRV_Status DRV_I2C_Init(const I2C_Config *cfg);

/**
 * @brief Write @p len bytes to register @p reg of the device at @p addr7.
 * @param addr7 7-bit slave address (the driver adds the R/W bit).
 * @param reg   8-bit register / index.
 * @return DRV_OK, DRV_INVALID_PARAM, or DRV_TIMEOUT on a stuck bus / no ACK.
 */
DRV_Status DRV_I2C_MemWrite(I2C_TypeDef *i2c, uint8_t addr7, uint8_t reg,
                            const uint8_t *data, uint16_t len);

/**
 * @brief Read @p len bytes from register @p reg of the device at @p addr7.
 * @param addr7 7-bit slave address.
 * @param reg   8-bit register / index.
 * @return DRV_OK, DRV_INVALID_PARAM, or DRV_TIMEOUT.
 *
 * Uses the standard write-index-then-repeated-start-read sequence.
 */
DRV_Status DRV_I2C_MemRead(I2C_TypeDef *i2c, uint8_t addr7, uint8_t reg,
                           uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* DRV_I2C_H_ */
