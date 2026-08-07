#ifndef DRV_TIMER_H_
#define DRV_TIMER_H_

#include "drv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef DRV_TIMER_PROVIDE_IRQ_HANDLERS
#define DRV_TIMER_PROVIDE_IRQ_HANDLERS 1
#endif

typedef void (*DRV_Timer_Callback)(TIM_TypeDef *tim, void *ctx);

DRV_Status DRV_Timer_InitHz(TIM_TypeDef *tim, uint32_t freq_hz);

DRV_Status DRV_Timer_InitRaw(TIM_TypeDef *tim, uint16_t psc, uint16_t arr);


void DRV_Timer_Start(TIM_TypeDef *tim);

void DRV_Timer_Stop(TIM_TypeDef *tim);


static inline uint16_t DRV_Timer_GetCounter(TIM_TypeDef *tim)
{
    return (uint16_t)tim->CNT;
}


static inline void DRV_Timer_SetCounter(TIM_TypeDef *tim, uint16_t value)
{
    tim->CNT = value;
}


DRV_Status DRV_Timer_AttachCallback(TIM_TypeDef *tim, DRV_Timer_Callback cb,
                                    void *ctx, uint32_t priority);


static inline void DRV_Timer_EnableUpdateIRQ(TIM_TypeDef *tim)
{
    DRV_SET_BITS(tim->DIER, TIM_DIER_UIE);
}

static inline void DRV_Timer_DisableUpdateIRQ(TIM_TypeDef *tim)
{
    DRV_CLEAR_BITS(tim->DIER, TIM_DIER_UIE);
}

void DRV_Timer_Dispatch(TIM_TypeDef *tim);

DRV_Status DRV_Timer_StartTick(TIM_TypeDef *tim, uint32_t priority);

uint32_t DRV_Timer_GetTick(void);

bool DRV_Timer_Elapsed(uint32_t start_ms, uint32_t timeout_ms);

DRV_Status DRV_Delay_Init(void);

void DRV_Delay_Us(uint32_t us);

void DRV_Delay_Ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif