#ifndef __DRV_PWM_H
#define __DRV_PWM_H

#include "stm32f10x.h"
#include <stdint.h>
#include "drv_gpio.h" 

typedef enum {
    PWM_TIM1, 
    PWM_TIM2, 
    PWM_TIM3, 
    PWM_TIM4  
} PWM_Timer;

typedef enum {
    PWM_CH1 = 1,
    PWM_CH2 = 2,
    PWM_CH3 = 3,
    PWM_CH4 = 4
} PWM_Channel;


void DRV_PWM_Init(PWM_Timer tim, Ports port, uint8_t pin, PWM_Channel channel, uint32_t frequency);

void DRV_PWM_Start(PWM_Timer tim, PWM_Channel channel);


void DRV_PWM_SetDuty(PWM_Timer tim, PWM_Channel channel, uint16_t duty);

void DRV_PWM_Stop(PWM_Timer tim, PWM_Channel channel);

#endif