/**
 ******************************************************************************
 * @file    drv_pwm.c
 * @brief   PWM output driver implementation (STM32F103).
 ******************************************************************************
 */
#include "drv_pwm.h"
#include "drv_gpio.h"
#include "drv_timer.h"

/* Address of the CCR register belonging to @p ch. */
static volatile uint32_t *pwm_ccr(TIM_TypeDef *tim, PWM_Channel ch)
{
    switch (ch) {
        case PWM_CH1: return &tim->CCR1;
        case PWM_CH2: return &tim->CCR2;
        case PWM_CH3: return &tim->CCR3;
        default:      return &tim->CCR4;
    }
}

/* PWM mode 1 + output preload for one channel. CH1/CH2 live in CCMR1 and
 * CH3/CH4 in CCMR2; within each register the field layout repeats every 8
 * bits, so one expression covers all four. */
static void pwm_set_mode(TIM_TypeDef *tim, PWM_Channel ch)
{
    const uint32_t shift = ((uint32_t)ch & 0x1u) * 8u;
    /* OCxM = 110 (PWM mode 1) | OCxPE (preload). Also clears CCxS so the
     * channel is an output. */
    const uint32_t value = (0x6u << 4) | (0x1u << 3);
    const uint32_t mask  = 0xFFu;

    if (ch == PWM_CH1 || ch == PWM_CH2) {
        DRV_MODIFY(tim->CCMR1, (mask << shift), (value << shift));
    } else {
        DRV_MODIFY(tim->CCMR2, (mask << shift), (value << shift));
    }
}

DRV_Status DRV_PWM_Init(const PWM_Config *cfg)
{
    if (cfg == NULL || cfg->tim == NULL || cfg->port == NULL ||
        cfg->pin > 15u || cfg->frequency == 0u) {
        return DRV_INVALID_PARAM;
    }

    /* Timebase: derived from the timer's real clock, not a hardcoded 72 MHz. */
    const DRV_Status st = DRV_Timer_InitHz(cfg->tim, cfg->frequency);
    if (st != DRV_OK) {
        return st;
    }

    /* The channel pin must be alternate-function push-pull. */
    DRV_GPIO_InitAF(cfg->port, cfg->pin);

    TIM_TypeDef *tim = cfg->tim;
    pwm_set_mode(tim, cfg->channel);

    /* CCER holds 4 bits per channel (CCxE, CCxP, ...). Set polarity, leave the
     * output disabled until DRV_PWM_Start(). */
    const uint32_t ccer_shift = (uint32_t)cfg->channel * 4u;
    uint32_t ccer = tim->CCER & ~(0xFu << ccer_shift);
    if (cfg->polarity == PWM_ACTIVE_LOW) {
        ccer |= (TIM_CCER_CC1P << ccer_shift);
    }
    tim->CCER = ccer;

    *pwm_ccr(tim, cfg->channel) = 0u;   /* start at 0 % */

    tim->EGR = TIM_EGR_UG;              /* load the new shadow registers */
    tim->SR  = (uint32_t)~TIM_SR_UIF;
    DRV_Timer_Start(tim);

    return DRV_OK;
}

void DRV_PWM_Start(TIM_TypeDef *tim, PWM_Channel channel)
{
    DRV_SET_BITS(tim->CCER, TIM_CCER_CC1E << ((uint32_t)channel * 4u));

    /* TIM1 is an advanced-control timer: its outputs stay high-impedance
     * until the main output enable is set. */
    if (tim == TIM1) {
        DRV_SET_BITS(tim->BDTR, TIM_BDTR_MOE);
    }
}

void DRV_PWM_Stop(TIM_TypeDef *tim, PWM_Channel channel)
{
    DRV_CLEAR_BITS(tim->CCER, TIM_CCER_CC1E << ((uint32_t)channel * 4u));
}

void DRV_PWM_SetCompare(TIM_TypeDef *tim, PWM_Channel channel,
                        uint16_t compare)
{
    *pwm_ccr(tim, channel) = compare;
}

void DRV_PWM_SetDuty(TIM_TypeDef *tim, PWM_Channel channel, uint8_t duty)
{
    if (duty > 100u) {
        duty = 100u;
    }
    /* Scale against the actual reload value so the duty is right whatever
     * frequency the timebase ended up at. */
    const uint32_t period = tim->ARR + 1u;
    *pwm_ccr(tim, channel) = (uint16_t)((period * duty) / 100u);
}
