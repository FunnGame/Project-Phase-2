#ifndef HAL_VL53_H
#define HAL_VL53_H

#include "types.h"
#include "error.h"

Status_t HAL_VL53_Init(void);

Status_t HAL_VL53_Start(void);

Status_t HAL_VL53_Stop(void);

Status_t HAL_VL53_IsDataReady(uint8 *ready);

Status_t HAL_VL53_ReadDistance(uint16 *distance);

Status_t HAL_VL53_ClearInterrupt(void);

#endif