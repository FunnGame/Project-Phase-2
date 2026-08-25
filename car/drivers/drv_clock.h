/**
 ******************************************************************************
 * @file    drv_clock.h
 * @brief   Clock configuration and introspection for the STM32F103.
 *
 * Two jobs:
 *   1. Bring SYSCLK up to a usable speed (the CMSIS SystemInit() for the F1
 *      does NOT configure the PLL — without this the part runs at 8 MHz HSI).
 *   2. Report the *actual* bus and timer clocks at runtime, so the timer, PWM
 *      and SPI drivers derive their dividers instead of assuming 72 MHz.
 *
 * Every other driver in this folder calls DRV_Clock_GetTimerClock() rather than
 * hardcoding a prescaler, which is what makes them correct at any SYSCLK.
 ******************************************************************************
 */
#ifndef DRV_CLOCK_H_
#define DRV_CLOCK_H_

#include "drv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configure SYSCLK to 72 MHz from an external @p hse_hz crystal.
 * @param hse_hz HSE frequency in Hz (8000000 for the common Blue Pill / most
 *               F103 boards). Pass 0 to stay on the 8 MHz HSI.
 * @return DRV_OK on success, DRV_TIMEOUT if the HSE or PLL fails to lock,
 *         DRV_UNSUPPORTED if @p hse_hz cannot reach 72 MHz.
 *
 * Sets flash latency, AHB /1 (72 MHz), APB1 /2 (36 MHz — the APB1 maximum) and
 * APB2 /1 (72 MHz), then updates SystemCoreClock.
 */
DRV_Status DRV_Clock_Init72MHz(uint32_t hse_hz);

/** @brief Current SYSCLK in Hz (refreshes SystemCoreClock from RCC). */
uint32_t DRV_Clock_GetSysClk(void);

/** @brief Current AHB / HCLK frequency in Hz. */
uint32_t DRV_Clock_GetHCLK(void);

/** @brief Current APB1 (PCLK1) frequency in Hz — max 36 MHz. */
uint32_t DRV_Clock_GetPCLK1(void);

/** @brief Current APB2 (PCLK2) frequency in Hz — max 72 MHz. */
uint32_t DRV_Clock_GetPCLK2(void);

/**
 * @brief Kernel clock feeding @p tim, in Hz.
 * @param tim TIM1..TIM4.
 * @return The timer's counting clock, accounting for the APB prescaler
 *         doubling rule (a timer runs at 2x PCLK when its APB divider > 1).
 *
 * This is the function that replaces "PSC = 71" style magic numbers.
 */
uint32_t DRV_Clock_GetTimerClock(TIM_TypeDef *tim);

/**
 * @brief Why the MCU last reset. Values match HBxx_ResetReason in adas.dbc.
 */
typedef enum {
    DRV_RESET_UNKNOWN   = 0,
    DRV_RESET_POWER_ON  = 1,   /**< POR/PDR - INCLUDES BROWN-OUT, see below  */
    DRV_RESET_PIN       = 2,   /**< NRST driven low                          */
    DRV_RESET_SOFTWARE  = 3,
    DRV_RESET_IWDG      = 4,
    DRV_RESET_WWDG      = 5,
    DRV_RESET_LOW_POWER = 6
} DRV_ResetReason;

/**
 * @brief Read and CLEAR the reset cause flags in RCC_CSR.
 */
DRV_ResetReason DRV_Clock_ResetReason(void);

#ifdef __cplusplus
}
#endif

#endif /* DRV_CLOCK_H_ */
