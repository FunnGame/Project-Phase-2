/**
 ******************************************************************************
 * @file    drv_pwm.h
 * @brief   PWM output driver for the STM32F103.
 *
 * Edge-aligned PWM on any timer/channel. The frequency is requested in HERTZ
 * and the prescaler/reload are derived from the timer's real kernel clock, so
 * the same call is correct at 8 MHz HSI and at 72 MHz PLL.
 *
 * Duty can be set as a percentage (simple) or in raw timer ticks (precise) —
 * the raw form matters at high PWM frequencies where 1 % is a coarse step.
 *
 * On the car this drives the TB6612 motor channels.
 ******************************************************************************
 */
#ifndef DRV_PWM_H_
#define DRV_PWM_H_

#include "drv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Capture/compare channel. */
typedef enum {
    PWM_CH1 = 0,
    PWM_CH2,
    PWM_CH3,
    PWM_CH4,
} PWM_Channel;

/** @brief Output polarity. */
typedef enum {
    PWM_ACTIVE_HIGH = 0,
    PWM_ACTIVE_LOW,
} PWM_Polarity;

/** @brief One PWM output. */
typedef struct {
    TIM_TypeDef  *tim;        /**< Timer generating the waveform.          */
    PWM_Channel   channel;    /**< Output channel.                         */
    GPIO_TypeDef *port;       /**< Port of the channel's output pin.       */
    uint8_t       pin;        /**< Pin number of the output.               */
    uint32_t      frequency;  /**< PWM frequency in Hz.                    */
    PWM_Polarity  polarity;   /**< Active level.                           */
} PWM_Config;

/**
 * @brief Configure the timebase, pin and channel described by @p cfg.
 * @return DRV_OK, DRV_INVALID_PARAM, or DRV_UNSUPPORTED if @p frequency is
 *         unreachable.
 *
 * Enables the clocks, sets the pin to alternate-function push-pull and puts
 * the channel in PWM mode 1 with output preload. The output starts at 0 % and
 * stays off until DRV_PWM_Start().
 *
 * Note: all channels of one timer share its timebase, so configuring a second
 * channel on the same timer with a different frequency overrides the first.
 */
DRV_Status DRV_PWM_Init(const PWM_Config *cfg);

/** @brief Enable the output of @p channel (and the main output for TIM1). */
void DRV_PWM_Start(TIM_TypeDef *tim, PWM_Channel channel);

/** @brief Disable the output of @p channel. */
void DRV_PWM_Stop(TIM_TypeDef *tim, PWM_Channel channel);

/**
 * @brief Set duty as a percentage.
 * @param duty 0..100 (clamped).
 */
void DRV_PWM_SetDuty(TIM_TypeDef *tim, PWM_Channel channel, uint8_t duty);

/**
 * @brief Set duty in raw timer ticks — full resolution.
 * @param compare 0..ARR; duty = compare / (ARR + 1).
 */
void DRV_PWM_SetCompare(TIM_TypeDef *tim, PWM_Channel channel,
                        uint16_t compare);

/**
 * @brief Reload value of @p tim, i.e. the maximum useful compare value.
 *
 * Use with DRV_PWM_SetCompare() to scale a control value without going through
 * an integer percentage.
 */
static inline uint16_t DRV_PWM_GetPeriod(TIM_TypeDef *tim)
{
    return (uint16_t)tim->ARR;
}

#ifdef __cplusplus
}
#endif

#endif /* DRV_PWM_H_ */
