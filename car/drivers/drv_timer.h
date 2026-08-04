/**
 ******************************************************************************
 * @file    drv_timer.h
 * @brief   General-purpose timer driver for the STM32F103.
 *
 * Periodic time bases and update-interrupt callbacks. Quadrature encoder
 * support lives in drv_encoder.h — the two used to share one file but have
 * nothing in common beyond the peripheral.
 *
 * The time base is requested in HERTZ, not in prescaler/reload magic numbers:
 * the driver reads the timer's real kernel clock via drv_clock and derives the
 * dividers itself. The same call is therefore correct at 8 MHz HSI and at
 * 72 MHz PLL.
 *
 * Any timer may carry a callback (a small registry backs the shared ISRs), so
 * the driver is no longer tied to TIM2.
 ******************************************************************************
 */
#ifndef DRV_TIMER_H_
#define DRV_TIMER_H_

#include "drv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Define as 0 to supply your own TIMx_IRQHandler and call DRV_Timer_Dispatch. */
#ifndef DRV_TIMER_PROVIDE_IRQ_HANDLERS
#define DRV_TIMER_PROVIDE_IRQ_HANDLERS 1
#endif

/**
 * @brief Update-event callback, invoked from the timer ISR.
 * @param tim The timer that raised the event.
 * @param ctx Opaque pointer supplied at registration.
 */
typedef void (*DRV_Timer_Callback)(TIM_TypeDef *tim, void *ctx);

/**
 * @brief Configure @p tim to overflow at @p freq_hz.
 * @param tim     TIM1..TIM4.
 * @param freq_hz Desired update frequency in Hz (> 0).
 * @return DRV_OK, DRV_INVALID_PARAM, or DRV_UNSUPPORTED when the frequency
 *         cannot be reached with a 16-bit prescaler/reload pair.
 *
 * Enables the peripheral clock and loads the shadow registers. The counter is
 * left stopped — call DRV_Timer_Start().
 */
DRV_Status DRV_Timer_InitHz(TIM_TypeDef *tim, uint32_t freq_hz);

/**
 * @brief Configure @p tim from raw prescaler/reload values.
 * @param psc Prescaler (0..65535); the timer counts at f_tim / (psc + 1).
 * @param arr Auto-reload (0..65535); it overflows every (arr + 1) counts.
 *
 * Use when you want exact register control; prefer DRV_Timer_InitHz otherwise.
 */
DRV_Status DRV_Timer_InitRaw(TIM_TypeDef *tim, uint16_t psc, uint16_t arr);

/** @brief Start the counter. */
void DRV_Timer_Start(TIM_TypeDef *tim);
/** @brief Stop the counter. */
void DRV_Timer_Stop(TIM_TypeDef *tim);

/** @brief Read the counter. */
static inline uint16_t DRV_Timer_GetCounter(TIM_TypeDef *tim)
{
    return (uint16_t)tim->CNT;
}

/** @brief Force the counter to @p value. */
static inline void DRV_Timer_SetCounter(TIM_TypeDef *tim, uint16_t value)
{
    tim->CNT = value;
}

/**
 * @brief Attach @p cb to @p tim's update event and enable its NVIC vector.
 * @param cb  Callback, or NULL to enable the interrupt without one.
 * @param ctx Opaque pointer handed back to @p cb.
 * @param priority NVIC priority (0 = highest).
 * @return DRV_OK, DRV_INVALID_PARAM, or DRV_ERROR if the registry is full.
 */
DRV_Status DRV_Timer_AttachCallback(TIM_TypeDef *tim, DRV_Timer_Callback cb,
                                    void *ctx, uint32_t priority);

/** @brief Enable the update interrupt without touching the NVIC. */
static inline void DRV_Timer_EnableUpdateIRQ(TIM_TypeDef *tim)
{
    DRV_SET_BITS(tim->DIER, TIM_DIER_UIE);
}

/** @brief Disable the update interrupt. */
static inline void DRV_Timer_DisableUpdateIRQ(TIM_TypeDef *tim)
{
    DRV_CLEAR_BITS(tim->DIER, TIM_DIER_UIE);
}

/**
 * @brief Clear @p tim's update flag and run its callback.
 * @note  Call from your own ISR when DRV_TIMER_PROVIDE_IRQ_HANDLERS is 0.
 */
void DRV_Timer_Dispatch(TIM_TypeDef *tim);

/* -------------------------------------------------------------------------- */
/*  Optional millisecond tick                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Start a 1 kHz tick on @p tim, counting milliseconds.
 * @param tim      Timer to dedicate to the tick.
 * @param priority NVIC priority.
 *
 * A convenience wrapper over InitHz + AttachCallback; replaces the old
 * hardcoded TIM2 tick.
 */
DRV_Status DRV_Timer_StartTick(TIM_TypeDef *tim, uint32_t priority);

/** @brief Milliseconds since DRV_Timer_StartTick(). Wraps after ~49 days. */
uint32_t DRV_Timer_GetTick(void);

/** @brief True once @p timeout_ms has elapsed since @p start_ms (wrap-safe). */
bool DRV_Timer_Elapsed(uint32_t start_ms, uint32_t timeout_ms);

/* -------------------------------------------------------------------------- */
/*  Blocking delays                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Enable the Cortex-M3 DWT cycle counter used by DRV_Delay_Us().
 * @return DRV_OK, or DRV_UNSUPPORTED if the counter refuses to run (some
 *         debug configurations gate it).
 *
 * Call once at startup, after the clock is configured.
 */
DRV_Status DRV_Delay_Init(void);

/**
 * @brief Block for @p us microseconds.
 *
 * Cycle-accurate via DWT and scaled from the real HCLK, so it stays correct
 * whatever SYSCLK you configured. Requires DRV_Delay_Init().
 */
void DRV_Delay_Us(uint32_t us);

/** @brief Block for @p ms milliseconds. */
void DRV_Delay_Ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* DRV_TIMER_H_ */
