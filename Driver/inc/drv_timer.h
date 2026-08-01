#ifndef DRV_TIMER_H
#define DRV_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32f10x.h"
#include "types.h"

typedef enum
{
    TIMER_1 = 0U,
    TIMER_2,
    TIMER_3,
    TIMER_4

} Timer_t;

Status_t DRV_TIMER_Init(Timer_t timer);

Status_t DRV_TIMER_TickInit(void);

Status_t DRV_SYSTICK_Init(void);

void DRV_DelayUs(uint32 us);

void DRV_DelayMs(uint32 ms);

uint32 DRV_TIMER_GetTick(void);

#ifdef __cplusplus
}
#endif

#endif