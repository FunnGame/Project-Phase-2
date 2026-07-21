/**
 ******************************************************************************
 * @file    drivers_common.h
 * @brief   Common definitions shared by all F446RE station drivers.
 *
 * This header centralises the MCU device selection, the CMSIS include, the
 * driver-wide status type and a handful of register-manipulation helpers.
 * Every station driver includes this file first so that the whole layer sees
 * a single, consistent set of primitives.
 *
 * Required compiler include paths:
 *   -Iplatform/f446    (device header: stm32f4xx.h / stm32f446xx.h)
 *   -Iplatform/cmsis   (core_cm4.h, cmsis_gcc.h)
 *
 * Target: STM32F446RE (Cortex-M4F, up to 180 MHz)
 ******************************************************************************
 */
#ifndef STATION_DRIVERS_COMMON_H
#define STATION_DRIVERS_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * The vendor CMSIS header (stm32f4xx.h) requires the exact device to be
 * selected via a preprocessor symbol. Define it here if the build system has
 * not already done so, so the drivers remain usable without extra -D flags.
 */
#if !defined(STM32F446xx)
#define STM32F446xx
#endif

#include "stm32f4xx.h"

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/*  Status / return codes                                                     */
/* -------------------------------------------------------------------------- */

/** @brief Uniform return code used across the whole driver layer. */
typedef enum {
    DRV_OK = 0,          /**< Operation completed successfully.            */
    DRV_ERROR,           /**< Generic, unspecified failure.                */
    DRV_BUSY,            /**< Peripheral busy, try again later.            */
    DRV_TIMEOUT,         /**< Operation timed out.                         */
    DRV_INVALID_PARAM,   /**< A supplied argument was out of range/NULL.   */
    DRV_UNSUPPORTED,     /**< Requested configuration is not supported.    */
} drv_status_t;

/* -------------------------------------------------------------------------- */
/*  Register / bit helpers                                                    */
/* -------------------------------------------------------------------------- */

/** @brief Build a single-bit mask for bit position @p n. */
#define DRV_BIT(n)                   (1UL << (n))

/** @brief Set every bit of @p mask in register @p reg. */
#define DRV_SET_BITS(reg, mask)      ((reg) |= (uint32_t)(mask))

/** @brief Clear every bit of @p mask in register @p reg. */
#define DRV_CLEAR_BITS(reg, mask)    ((reg) &= ~(uint32_t)(mask))

/** @brief Read the bits of @p mask from register @p reg. */
#define DRV_READ_BITS(reg, mask)     ((reg) & (uint32_t)(mask))

/**
 * @brief Atomically (from the reader's point of view) clear @p clr_mask and
 *        then set @p set_mask inside register @p reg.
 */
#define DRV_MODIFY(reg, clr_mask, set_mask) \
    ((reg) = (((reg) & ~(uint32_t)(clr_mask)) | (uint32_t)(set_mask)))

/** @brief Suppress "unused parameter" warnings for optional arguments. */
#define DRV_UNUSED(x)                ((void)(x))

/** @brief Number of elements in a statically-sized array. */
#define DRV_ARRAY_LEN(a)             (sizeof(a) / sizeof((a)[0]))

/* -------------------------------------------------------------------------- */
/*  Polling helper                                                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Poll a status register until a masked field matches @p expected.
 * @param reg      Pointer to the (volatile) register to read.
 * @param mask     Bits of interest.
 * @param expected Required value of (reg & mask), e.g. @p mask to wait for a
 *                 flag to set, or 0 to wait for it to clear.
 * @param loops    Busy-wait iteration budget (not wall-clock time).
 * @return DRV_OK once the condition holds, DRV_TIMEOUT if @p loops elapse.
 *
 * For time-based waits, prefer the systick driver.
 */
static inline drv_status_t drv_wait_flag(volatile const uint32_t *reg,
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
/*  Generic NVIC helpers (usable by any driver)                               */
/* -------------------------------------------------------------------------- */

/** @brief Set priority and enable @p irqn in the NVIC. */
static inline void nvic_enable(IRQn_Type irqn, uint32_t priority)
{
    NVIC_SetPriority(irqn, priority);
    NVIC_EnableIRQ(irqn);
}

/** @brief Disable @p irqn in the NVIC. */
static inline void nvic_disable(IRQn_Type irqn)
{
    NVIC_DisableIRQ(irqn);
}

#ifdef __cplusplus
}
#endif

#endif /* STATION_DRIVERS_COMMON_H */
