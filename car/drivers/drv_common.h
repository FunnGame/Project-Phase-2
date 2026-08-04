/**
 ******************************************************************************
 * @file    drv_common.h
 * @brief   Definitions shared by every car (STM32F103) driver.
 *
 * Centralises the device selection, the CMSIS include, the driver-wide status
 * type and the register helpers, so no driver repeats that boilerplate and all
 * of them agree on the same primitives.
 *
 * Required include paths:
 *   -Icar/drivers  -Iplatform/f103  -Iplatform/cmsis
 *
 * Target: STM32F103C8 (Cortex-M3, 72 MHz max)
 ******************************************************************************
 */
#ifndef DRV_COMMON_H_
#define DRV_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

/* The CMSIS device header needs the exact part selected. Define it here if the
 * build system has not, so the drivers work without extra -D flags. */
#if !defined(STM32F103xB)
#define STM32F103xB
#endif

#include "stm32f1xx.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/*  Status codes                                                              */
/* -------------------------------------------------------------------------- */

/** @brief Uniform return code used across the car driver layer. */
typedef enum {
    DRV_OK = 0,          /**< Completed successfully.                      */
    DRV_ERROR,           /**< Generic failure.                             */
    DRV_BUSY,            /**< Peripheral busy.                             */
    DRV_TIMEOUT,         /**< Operation timed out.                         */
    DRV_INVALID_PARAM,   /**< Argument out of range / NULL.                */
    DRV_UNSUPPORTED,     /**< Requested configuration not supported.       */
} DRV_Status;

/* -------------------------------------------------------------------------- */
/*  Register helpers                                                          */
/* -------------------------------------------------------------------------- */

#define DRV_BIT(n)                  (1UL << (n))
#define DRV_SET_BITS(reg, mask)     ((reg) |= (uint32_t)(mask))
#define DRV_CLEAR_BITS(reg, mask)   ((reg) &= ~(uint32_t)(mask))
#define DRV_READ_BITS(reg, mask)    ((reg) & (uint32_t)(mask))

/** @brief Clear @p clr then set @p set inside register @p reg. */
#define DRV_MODIFY(reg, clr, set) \
    ((reg) = (((reg) & ~(uint32_t)(clr)) | (uint32_t)(set)))

#define DRV_UNUSED(x)               ((void)(x))
#define DRV_ARRAY_LEN(a)            (sizeof(a) / sizeof((a)[0]))

/**
 * @brief Poll a register until a masked field equals @p expected.
 * @param reg      Register to read.
 * @param mask     Bits of interest.
 * @param expected Required value of (reg & mask): @p mask to wait for a flag to
 *                 set, 0 to wait for it to clear.
 * @param loops    Busy-wait iteration budget (not wall-clock time).
 * @return DRV_OK once the condition holds, DRV_TIMEOUT if the budget elapses.
 */
static inline DRV_Status DRV_WaitFlag(volatile const uint32_t *reg,
                                      uint32_t mask, uint32_t expected,
                                      uint32_t loops)
{
    for (volatile uint32_t i = 0; i < loops; ++i) {
        if ((*reg & mask) == expected) {
            return DRV_OK;
        }
    }
    return DRV_TIMEOUT;
}

/* -------------------------------------------------------------------------- */
/*  Shared GPIO port helpers                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Enable the APB2 clock for @p port (GPIOA..GPIOE).
 *
 * Lives here rather than in drv_gpio so the PWM and SPI drivers can enable a
 * port clock without depending on the whole GPIO driver.
 */
static inline void DRV_GPIO_ClockEnable(GPIO_TypeDef *port)
{
    if      (port == GPIOA) DRV_SET_BITS(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);
    else if (port == GPIOB) DRV_SET_BITS(RCC->APB2ENR, RCC_APB2ENR_IOPBEN);
    else if (port == GPIOC) DRV_SET_BITS(RCC->APB2ENR, RCC_APB2ENR_IOPCEN);
    else if (port == GPIOD) DRV_SET_BITS(RCC->APB2ENR, RCC_APB2ENR_IOPDEN);
    else if (port == GPIOE) DRV_SET_BITS(RCC->APB2ENR, RCC_APB2ENR_IOPEEN);
}

/**
 * @brief Write the 4-bit CRL/CRH configuration nibble for one pin.
 * @param port   GPIO port.
 * @param pin    Pin number 0..15.
 * @param nibble [CNF(2) | MODE(2)] value for that pin.
 *
 * Touches only the four bits belonging to @p pin, so configuring one pin never
 * disturbs its neighbours.
 */
static inline void DRV_GPIO_WriteCfgNibble(GPIO_TypeDef *port, uint8_t pin,
                                           uint8_t nibble)
{
    volatile uint32_t *cr = (pin < 8u) ? &port->CRL : &port->CRH;
    const uint32_t shift = (uint32_t)(pin & 0x7u) * 4u;
    DRV_MODIFY(*cr, (0xFUL << shift), ((uint32_t)(nibble & 0xFu) << shift));
}

#ifdef __cplusplus
}
#endif

#endif /* DRV_COMMON_H_ */
