#ifndef ACC_H_
#define ACC_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void ACC_Init(void);

void ACC_Run(uint16_t distance_mm, float distance_dt, uint16_t active_distance_mm, float setpoint_rpm);

#ifdef __cplusplus
}
#endif

#endif