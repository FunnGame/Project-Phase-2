#ifndef DRV_PWM_H_
#define DRV_PWM_H_

#include "drv_common.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef enum {
    PWM_CH1 = 0,
    PWM_CH2,
    PWM_CH3,
    PWM_CH4,
} PWM_Channel;


typedef enum {
    PWM_ACTIVE_HIGH = 0,
    PWM_ACTIVE_LOW,
} PWM_Polarity;


typedef struct {
    TIM_TypeDef  *tim;        
    PWM_Channel   channel;    
    GPIO_TypeDef *port;       
    uint8_t       pin;        
    uint32_t      frequency; 
    PWM_Polarity  polarity;   
} PWM_Config;


DRV_Status DRV_PWM_Init(const PWM_Config *cfg);


void DRV_PWM_Start(TIM_TypeDef *tim, PWM_Channel channel);


void DRV_PWM_Stop(TIM_TypeDef *tim, PWM_Channel channel);


void DRV_PWM_SetDuty(TIM_TypeDef *tim, PWM_Channel channel, uint8_t duty);


void DRV_PWM_SetCompare(TIM_TypeDef *tim, PWM_Channel channel,
                        uint16_t compare);


static inline uint16_t DRV_PWM_GetPeriod(TIM_TypeDef *tim)
{
    return (uint16_t)tim->ARR;
}

#ifdef __cplusplus
}
#endif

#endif 