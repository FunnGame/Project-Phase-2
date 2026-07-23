#include "drv_pwm.h"

static TIM_TypeDef* Get_Timer_Instance(PWM_Timer tim) {
    switch (tim) {
        case PWM_TIM1: return TIM1;
        case PWM_TIM2: return TIM2;
        case PWM_TIM3: return TIM3;
        case PWM_TIM4: return TIM4;
        default: return TIM2;
    }
}

void DRV_PWM_Init(PWM_Timer tim, Ports port, uint8_t pin, PWM_Channel channel, uint32_t frequency) {
    RCC->APB2ENR |= (1 << (port + 2));

    if (tim == PWM_TIM1) {
        RCC->APB2ENR |= (1 << 11);
    } else {
        RCC->APB1ENR |= (1 << (tim - 1));
    }

    GPIO_TypeDef *gpio_port;
    if (port == PA)       gpio_port = GPIOA;
    else if (port == PB)  gpio_port = GPIOB;
    else                  gpio_port = GPIOC;

    uint8_t bit_pos = (pin % 8) * 4;
    if (pin < 8) {
        gpio_port->CRL &= ~(0x0FUL << bit_pos);
        gpio_port->CRL |= (0x0AUL << bit_pos);
    } else {
        gpio_port->CRH &= ~(0x0FUL << bit_pos);
        gpio_port->CRH |= (0x0AUL << bit_pos);
    }

    TIM_TypeDef *TIMx = Get_Timer_Instance(tim);
    TIMx->PSC = 71;
    TIMx->ARR = (1000000UL / frequency) - 1;

    switch (channel) {
        case PWM_CH1:
            TIMx->CCMR1 &= ~(0x7UL << 4);
            TIMx->CCMR1 |= (0x6UL << 4) | (1UL << 3);
            break;
        case PWM_CH2:
            TIMx->CCMR1 &= ~(0x7UL << 12);
            TIMx->CCMR1 |= (0x6UL << 12) | (1UL << 11);
            break;
        case PWM_CH3:
            TIMx->CCMR2 &= ~(0x7UL << 4);
            TIMx->CCMR2 |= (0x6UL << 4) | (1UL << 3);
            break;
        case PWM_CH4:
            TIMx->CCMR2 &= ~(0x7UL << 12);
            TIMx->CCMR2 |= (0x6UL << 12) | (1UL << 11);
            break;
    }

    TIMx->CR1 |= (1 << 7);
    TIMx->CR1 |= (1 << 0);
}

void DRV_PWM_Start(PWM_Timer tim, PWM_Channel channel) {
    TIM_TypeDef *TIMx = Get_Timer_Instance(tim);

    switch (channel) {
        case PWM_CH1: TIMx->CCER |= (1 << 0);  break;
        case PWM_CH2: TIMx->CCER |= (1 << 4);  break;
        case PWM_CH3: TIMx->CCER |= (1 << 8);  break;
        case PWM_CH4: TIMx->CCER |= (1 << 12); break;
    }

    if (tim == PWM_TIM1) {
        TIMx->BDTR |= (1 << 15);
    }
}

void DRV_PWM_SetDuty(PWM_Timer tim, PWM_Channel channel, uint16_t duty) {
    if (duty > 100) duty = 100;

    TIM_TypeDef *TIMx = Get_Timer_Instance(tim);

    uint32_t ccr_value = ((TIMx->ARR + 1) * duty) / 100;

    switch (channel) {
        case PWM_CH1: TIMx->CCR1 = ccr_value; break;
        case PWM_CH2: TIMx->CCR2 = ccr_value; break;
        case PWM_CH3: TIMx->CCR3 = ccr_value; break;
        case PWM_CH4: TIMx->CCR4 = ccr_value; break;
    }
}

void DRV_PWM_Stop(PWM_Timer tim, PWM_Channel channel) {
    TIM_TypeDef *TIMx = Get_Timer_Instance(tim);

    switch (channel) {
        case PWM_CH1: TIMx->CCER &= ~(1 << 0);  break;
        case PWM_CH2: TIMx->CCER &= ~(1 << 4);  break;
        case PWM_CH3: TIMx->CCER &= ~(1 << 8);  break;
        case PWM_CH4: TIMx->CCER &= ~(1 << 12); break;
    }
}