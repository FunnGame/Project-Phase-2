#include "drv_encoder.h"

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

    DRV_GPIO_InitInput(cfg->port_a, cfg->pin_a, GPIO_PULL_NONE);
    DRV_GPIO_InitInput(cfg->port_b, cfg->pin_b, GPIO_PULL_NONE);

    TIM_TypeDef *tim = cfg->tim;

    tim->CR1 &= ~TIM_CR1_CEN;          
    tim->PSC = 0u;                     
    tim->ARR = cfg->arr;


    tim->CCMR1 = (0x1u << 0) |
             (0x1u << 8);

    
    tim->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P);
    if (cfg->invert) {
        tim->CCER |= TIM_CCER_CC1P;
    }

    
    DRV_MODIFY(tim->SMCR, TIM_SMCR_SMS, ((uint32_t)cfg->mode & 0x7u));

    tim->CNT = 0u;
    tim->EGR = TIM_EGR_UG;             
    tim->SR  = (uint32_t)~TIM_SR_UIF;  
    tim->CR1 |= TIM_CR1_CEN;

    return DRV_OK;
}

int16_t DRV_Encoder_GetDelta(TIM_TypeDef *tim, uint16_t *prev_count)
{
    const uint16_t now = (uint16_t)tim->CNT;
    const int16_t delta = (int16_t)(uint16_t)(now - *prev_count);
    *prev_count = now;
    return delta;
}