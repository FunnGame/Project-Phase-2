/**
 ******************************************************************************
 * @file    drv_encoder.c
 * @brief   Quadrature encoder driver implementation (STM32F103).
 ******************************************************************************
 */
#include "drv_encoder.h"

/* Enable the peripheral clock; false for an instance we do not know. */
static bool encoder_clock_enable(TIM_TypeDef *tim)
{
    if      (tim == TIM1) DRV_SET_BITS(RCC->APB2ENR, RCC_APB2ENR_TIM1EN);
    else if (tim == TIM2) DRV_SET_BITS(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
    else if (tim == TIM3) DRV_SET_BITS(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
    else if (tim == TIM4) DRV_SET_BITS(RCC->APB1ENR, RCC_APB1ENR_TIM4EN);
    else return false;
    return true;
}

DRV_Status DRV_Encoder_Init(const Encoder_Config *cfg)
{
    if (cfg == NULL || cfg->tim == NULL ||
        cfg->port_a == NULL || cfg->port_b == NULL ||
        cfg->pin_a > 15u || cfg->pin_b > 15u) {
        return DRV_INVALID_PARAM;
    }
    if (!encoder_clock_enable(cfg->tim)) {
        return DRV_INVALID_PARAM;
    }

    /* Both phases are digital inputs. Floating suits an encoder that drives
     * the line actively; use DRV_GPIO_InitInput(.., GPIO_PULL_UP) instead for
     * an open-collector output. */
    DRV_GPIO_InitInput(cfg->port_a, cfg->pin_a, GPIO_PULL_NONE);
    DRV_GPIO_InitInput(cfg->port_b, cfg->pin_b, GPIO_PULL_NONE);

    TIM_TypeDef *tim = cfg->tim;

    tim->CR1 &= ~TIM_CR1_CEN;          /* stop while reconfiguring */
    tim->PSC = 0u;                     /* never prescale an encoder */
    tim->ARR = cfg->arr;

    /* Map CH1 -> TI1 and CH2 -> TI2 (CC1S = CC2S = 01), no input filter. */
    tim->CCMR1 = (0x1u << TIM_CCMR1_CC1S_Pos) | (0x1u << TIM_CCMR1_CC2S_Pos);

    /* Both inputs non-inverted; optionally flip CC1P to reverse direction. */
    tim->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    if (cfg->invert) {
        tim->CCER |= TIM_CCER_CC1P;
    }

    /* Slave mode = encoder mode 1/2/3. */
    DRV_MODIFY(tim->SMCR, TIM_SMCR_SMS, ((uint32_t)cfg->mode & 0x7u));

    tim->CNT = 0u;
    tim->EGR = TIM_EGR_UG;             /* load PSC/ARR shadow registers */
    tim->SR  = (uint32_t)~TIM_SR_UIF;  /* drop the flag that raised */
    tim->CR1 |= TIM_CR1_CEN;

    return DRV_OK;
}

int16_t DRV_Encoder_GetDelta(TIM_TypeDef *tim, uint16_t *prev_count)
{
    const uint16_t now = (uint16_t)tim->CNT;
    /* Unsigned subtraction wraps correctly; the cast to int16_t then gives a
     * signed delta that is right for both directions across an overflow. */
    const int16_t delta = (int16_t)(uint16_t)(now - *prev_count);
    *prev_count = now;
    return delta;
}
