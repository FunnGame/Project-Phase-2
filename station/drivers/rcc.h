/**
 ******************************************************************************
 * @file    rcc.h
 * @brief   Reset & Clock Control driver for the STM32F446RE.
 *
 * Provides:
 *   - Thin, type-safe peripheral clock enable/disable/reset helpers that wrap
 *     the AHB1/AHB2/APB1/APB2 enable registers.
 *   - An optional one-call system-clock bring-up (HSE + main PLL) to a
 *     user-chosen SYSCLK, with sensible defaults for a Nucleo-F446RE.
 *   - Runtime getters for SYSCLK / HCLK / PCLK1 / PCLK2 that other drivers
 *     (timer, uart, spi) use to derive dividers and baud rates.
 *
 * The enable/reset helpers take the bit masks straight from the CMSIS device
 * header (e.g. RCC_AHB1ENR_GPIOAEN, RCC_APB1ENR_USART2EN), so any peripheral
 * on the part can be driven without extending this driver.
 ******************************************************************************
 */
#ifndef STATION_DRIVERS_RCC_H
#define STATION_DRIVERS_RCC_H

#include "drivers_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/*  Peripheral clock gating                                                   */
/* -------------------------------------------------------------------------- */

/** @brief Enable one or more AHB1 peripheral clocks (RCC_AHB1ENR_* masks). */
static inline void rcc_ahb1_enable(uint32_t mask) { DRV_SET_BITS(RCC->AHB1ENR, mask); (void)RCC->AHB1ENR; }
/** @brief Disable one or more AHB1 peripheral clocks. */
static inline void rcc_ahb1_disable(uint32_t mask) { DRV_CLEAR_BITS(RCC->AHB1ENR, mask); }

/** @brief Enable one or more AHB2 peripheral clocks (RCC_AHB2ENR_* masks). */
static inline void rcc_ahb2_enable(uint32_t mask) { DRV_SET_BITS(RCC->AHB2ENR, mask); (void)RCC->AHB2ENR; }
/** @brief Disable one or more AHB2 peripheral clocks. */
static inline void rcc_ahb2_disable(uint32_t mask) { DRV_CLEAR_BITS(RCC->AHB2ENR, mask); }

/** @brief Enable one or more APB1 peripheral clocks (RCC_APB1ENR_* masks). */
static inline void rcc_apb1_enable(uint32_t mask) { DRV_SET_BITS(RCC->APB1ENR, mask); (void)RCC->APB1ENR; }
/** @brief Disable one or more APB1 peripheral clocks. */
static inline void rcc_apb1_disable(uint32_t mask) { DRV_CLEAR_BITS(RCC->APB1ENR, mask); }

/** @brief Enable one or more APB2 peripheral clocks (RCC_APB2ENR_* masks). */
static inline void rcc_apb2_enable(uint32_t mask) { DRV_SET_BITS(RCC->APB2ENR, mask); (void)RCC->APB2ENR; }
/** @brief Disable one or more APB2 peripheral clocks. */
static inline void rcc_apb2_disable(uint32_t mask) { DRV_CLEAR_BITS(RCC->APB2ENR, mask); }

/** @brief Pulse the AHB1 reset line(s) for the given peripheral mask. */
static inline void rcc_ahb1_reset(uint32_t mask) { DRV_SET_BITS(RCC->AHB1RSTR, mask); DRV_CLEAR_BITS(RCC->AHB1RSTR, mask); }
/** @brief Pulse the APB1 reset line(s) for the given peripheral mask. */
static inline void rcc_apb1_reset(uint32_t mask) { DRV_SET_BITS(RCC->APB1RSTR, mask); DRV_CLEAR_BITS(RCC->APB1RSTR, mask); }
/** @brief Pulse the APB2 reset line(s) for the given peripheral mask. */
static inline void rcc_apb2_reset(uint32_t mask) { DRV_SET_BITS(RCC->APB2RSTR, mask); DRV_CLEAR_BITS(RCC->APB2RSTR, mask); }

/**
 * @brief Enable the AHB1 GPIO clock that owns @p port.
 * @param port One of GPIOA .. GPIOH.
 */
void rcc_gpio_clock_enable(GPIO_TypeDef *port);

/* -------------------------------------------------------------------------- */
/*  System clock configuration                                                */
/* -------------------------------------------------------------------------- */

/** @brief Source used to drive the main PLL / SYSCLK. */
typedef enum {
    RCC_SYSCLK_SRC_HSI = 0,  /**< 16 MHz internal RC oscillator.           */
    RCC_SYSCLK_SRC_HSE,      /**< External oscillator/clock (see hse_hz).  */
} rcc_sysclk_src_t;

/** @brief APB bus prescaler options (HCLK -> PCLKx). */
typedef enum {
    RCC_APB_DIV1 = 0,
    RCC_APB_DIV2,
    RCC_APB_DIV4,
    RCC_APB_DIV8,
    RCC_APB_DIV16,
} rcc_apb_div_t;

/**
 * @brief System-clock bring-up configuration.
 *
 * The main PLL is configured as:
 *   VCO_in  = src_hz / pll_m           (must be 1..2 MHz, 2 MHz recommended)
 *   VCO_out = VCO_in * pll_n           (must be 100..432 MHz)
 *   SYSCLK  = VCO_out / pll_p          (pll_p in {2,4,6,8})
 * with HCLK = SYSCLK / ahb_div and PCLKx = HCLK / apbX_div.
 *
 * Use rcc_sysclk_config_default() for a ready-made 180 MHz profile.
 */
typedef struct {
    rcc_sysclk_src_t src;    /**< PLL input source.                        */
    uint32_t hse_hz;         /**< HSE frequency in Hz (ignored for HSI).   */
    uint32_t pll_m;          /**< PLL /M divider (2..63).                   */
    uint32_t pll_n;          /**< PLL xN multiplier (50..432).             */
    uint32_t pll_p;          /**< PLL /P divider: 2, 4, 6 or 8.            */
    uint32_t ahb_div;        /**< HCLK divider: 1,2,4,...,512.             */
    rcc_apb_div_t apb1_div;  /**< PCLK1 divider (APB1 max 45 MHz).         */
    rcc_apb_div_t apb2_div;  /**< PCLK2 divider (APB2 max 90 MHz).         */
    uint32_t flash_ws;       /**< Flash wait states (0..15).               */
} rcc_sysclk_config_t;

/**
 * @brief Bring up SYSCLK from the supplied PLL configuration.
 * @param cfg Fully-populated configuration (must not be NULL).
 * @return DRV_OK on success, DRV_TIMEOUT if an oscillator/PLL fails to lock,
 *         DRV_INVALID_PARAM on a NULL/illegal configuration.
 *
 * On success SystemCoreClock and the cached bus frequencies are updated.
 */
drv_status_t rcc_sysclk_config(const rcc_sysclk_config_t *cfg);

/**
 * @brief Configure SYSCLK to 180 MHz from an @p hse_hz external oscillator.
 * @param hse_hz HSE frequency in Hz, or 0 to use the 16 MHz HSI instead.
 * @return See rcc_sysclk_config().
 *
 * For a Nucleo-F446RE driven by the ST-Link 8 MHz MCO, pass 8000000.
 */
drv_status_t rcc_sysclk_config_default(uint32_t hse_hz);

/* -------------------------------------------------------------------------- */
/*  Clock introspection                                                       */
/* -------------------------------------------------------------------------- */

/** @brief Recompute and return SYSCLK in Hz (also refreshes SystemCoreClock). */
uint32_t rcc_get_sysclk_hz(void);
/** @brief Current AHB / HCLK frequency in Hz. */
uint32_t rcc_get_hclk_hz(void);
/** @brief Current APB1 (PCLK1) frequency in Hz. */
uint32_t rcc_get_pclk1_hz(void);
/** @brief Current APB2 (PCLK2) frequency in Hz. */
uint32_t rcc_get_pclk2_hz(void);
/**
 * @brief Clock feeding an APB1 timer (PCLK1, doubled when APB1 is prescaled).
 * @return Timer kernel clock in Hz for TIM2..TIM7, TIM12..TIM14.
 */
uint32_t rcc_get_timer_pclk1_hz(void);
/**
 * @brief Clock feeding an APB2 timer (PCLK2, doubled when APB2 is prescaled).
 * @return Timer kernel clock in Hz for TIM1, TIM8..TIM11.
 */
uint32_t rcc_get_timer_pclk2_hz(void);

#ifdef __cplusplus
}
#endif

#endif /* STATION_DRIVERS_RCC_H */
