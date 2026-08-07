#ifndef DRV_CLOCK_H_
#define DRV_CLOCK_H_

#include "drv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

DRV_Status DRV_Clock_Init72MHz(uint32_t hse_hz);

uint32_t DRV_Clock_GetSysClk(void);

uint32_t DRV_Clock_GetHCLK(void);

uint32_t DRV_Clock_GetPCLK1(void);

uint32_t DRV_Clock_GetPCLK2(void);

uint32_t DRV_Clock_GetTimerClock(TIM_TypeDef *tim);

#ifdef __cplusplus
}
#endif

#endif 