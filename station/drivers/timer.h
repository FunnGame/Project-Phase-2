/**
 ******************************************************************************
 * @file    timer.h
 * @brief   General-purpose timer driver for the STM32F446RE.
 *
 * Covers the three most common station use-cases with one small API:
 *   1. Periodic time base + update interrupt (scheduler ticks, timeouts).
 *   2. PWM generation (e.g. TFT backlight dimming).
 *   3. Free-running counter for coarse timestamping / us delays.
 *
 * The time base is specified either directly (prescaler + period register
 * values) or in Hz via timer_init_hz(), which derives the dividers from the
 * timer's actual kernel clock (queried from the rcc driver). This keeps the
 * driver correct regardless of the APB prescaler settings.
 *
 * IRQ wiring mirrors the exti driver: define TIMER_PROVIDE_IRQ_HANDLERS as 0
 * to supply your own vectors and call timer_irq_dispatch() from them.
 ******************************************************************************
 */
#ifndef STATION_DRIVERS_TIMER_H
#define STATION_DRIVERS_TIMER_H

#include "drivers_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TIMER_PROVIDE_IRQ_HANDLERS
#define TIMER_PROVIDE_IRQ_HANDLERS 1
#endif

/** @brief Counting direction (basic timers are up-only). */
typedef enum {
    TIMER_COUNT_UP = 0,
    TIMER_COUNT_DOWN,
} timer_direction_t;

/** @brief Output-compare capture/compare channel. */
typedef enum {
    TIMER_CH1 = 0,
    TIMER_CH2,
    TIMER_CH3,
    TIMER_CH4,
} timer_channel_t;

/** @brief PWM output polarity. */
typedef enum {
    TIMER_PWM_ACTIVE_HIGH = 0,
    TIMER_PWM_ACTIVE_LOW,
} timer_pwm_polarity_t;

/**
 * @brief Update-event callback, invoked from the timer ISR.
 * @param tim The timer instance that raised the update event.
 * @param ctx Opaque user pointer from timer_attach_update_irq().
 */
typedef void (*timer_callback_t)(TIM_TypeDef *tim, void *ctx);

/**
 * @brief Time-base configuration.
 *
 * Update frequency = f_tim / ((prescaler + 1) * (period + 1)), where f_tim is
 * the timer kernel clock. @p period is written to ARR, @p prescaler to PSC.
 */
typedef struct {
    uint32_t          prescaler;    /**< PSC value (0..65535).             */
    uint32_t          period;       /**< ARR value (0..65535, or 32-bit
                                         for TIM2/TIM5).                    */
    timer_direction_t direction;    /**< Count direction.                  */
    bool              auto_reload_preload; /**< Buffer ARR (ARPE).         */
    bool              one_pulse;    /**< Stop after one update (OPM).      */
} timer_base_config_t;

/* -------------------------------------------------------------------------- */
/*  Time base                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Enable @p tim's clock and program its time base from @p cfg.
 * @return DRV_OK or DRV_INVALID_PARAM.
 * @note   The counter is left stopped; call timer_start().
 */
drv_status_t timer_init(TIM_TypeDef *tim, const timer_base_config_t *cfg);

/**
 * @brief Configure @p tim to raise an update event at @p hz hertz.
 * @param tim TIM instance.
 * @param hz  Desired update frequency in Hz (must be > 0).
 * @return DRV_OK, DRV_INVALID_PARAM, or DRV_UNSUPPORTED if @p hz cannot be
 *         reached with a 16-bit prescaler/period pair.
 *
 * The prescaler/period split is chosen automatically from the timer's kernel
 * clock so the same call works on APB1 and APB2 timers.
 */
drv_status_t timer_init_hz(TIM_TypeDef *tim, uint32_t hz);

/** @brief Start (enable) the counter. */
void timer_start(TIM_TypeDef *tim);
/** @brief Stop (disable) the counter. */
void timer_stop(TIM_TypeDef *tim);
/** @brief Read the current counter value. */
static inline uint32_t timer_get_counter(TIM_TypeDef *tim) { return tim->CNT; }
/** @brief Force the counter to @p value. */
static inline void timer_set_counter(TIM_TypeDef *tim, uint32_t value) { tim->CNT = value; }
/** @brief Change the auto-reload (period) register on the fly. */
static inline void timer_set_period(TIM_TypeDef *tim, uint32_t period) { tim->ARR = period; }
/** @brief Change the prescaler; takes effect at the next update event. */
static inline void timer_set_prescaler(TIM_TypeDef *tim, uint32_t psc) { tim->PSC = psc; }

/* -------------------------------------------------------------------------- */
/*  Update interrupt                                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Attach @p cb to @p tim's update event and enable its NVIC vector.
 * @param tim      TIM instance.
 * @param cb       Callback, or NULL to just enable the update interrupt.
 * @param ctx      Opaque pointer passed back to @p cb.
 * @param priority NVIC priority (0 = highest).
 * @return DRV_OK or DRV_INVALID_PARAM (unsupported timer instance).
 */
drv_status_t timer_attach_update_irq(TIM_TypeDef *tim, timer_callback_t cb,
                                     void *ctx, uint32_t priority);

/** @brief Enable the update (UEV) interrupt without touching the NVIC. */
static inline void timer_enable_update_irq(TIM_TypeDef *tim)  { DRV_SET_BITS(tim->DIER, TIM_DIER_UIE); }
/** @brief Disable the update (UEV) interrupt. */
static inline void timer_disable_update_irq(TIM_TypeDef *tim) { DRV_CLEAR_BITS(tim->DIER, TIM_DIER_UIE); }

/**
 * @brief Service handler for @p tim; clears the update flag and calls the cb.
 * @note  Call from a custom ISR when TIMER_PROVIDE_IRQ_HANDLERS is 0.
 */
void timer_irq_dispatch(TIM_TypeDef *tim);

/* -------------------------------------------------------------------------- */
/*  PWM output                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Configure @p channel of @p tim as an edge-aligned PWM output.
 * @param tim      TIM instance (time base must already be set).
 * @param channel  Output channel.
 * @param polarity Active level.
 * @return DRV_OK or DRV_INVALID_PARAM.
 * @note   The matching GPIO pin must be put in AF mode separately, and for the
 *         advanced timer TIM1 the main output enable (BDTR.MOE) is set here.
 */
drv_status_t timer_pwm_config(TIM_TypeDef *tim, timer_channel_t channel,
                              timer_pwm_polarity_t polarity);

/**
 * @brief Set the compare (duty) value of @p channel.
 * @param compare Duty in timer ticks; duty% = compare / (ARR + 1).
 */
void timer_pwm_set_compare(TIM_TypeDef *tim, timer_channel_t channel,
                           uint32_t compare);

/** @brief Enable output on @p channel (CCER capture/compare enable). */
void timer_channel_enable(TIM_TypeDef *tim, timer_channel_t channel);
/** @brief Disable output on @p channel. */
void timer_channel_disable(TIM_TypeDef *tim, timer_channel_t channel);

#ifdef __cplusplus
}
#endif

#endif /* STATION_DRIVERS_TIMER_H */
