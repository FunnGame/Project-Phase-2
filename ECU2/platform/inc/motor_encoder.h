#ifndef MOTOR_ENCODER_H_
#define MOTOR_ENCODER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void MotorEncoder_Init(void);

void MotorEncoder_Process(void);

int16_t MotorEncoder_GetLeftDelta(void);
int16_t MotorEncoder_GetRightDelta(void);


float MotorEncoder_GetLeftRPM(void);
float MotorEncoder_GetRightRPM(void);


int32_t MotorEncoder_GetLeftTicks(void);
int32_t MotorEncoder_GetRightTicks(void);


void MotorEncoder_ResetTicks(void);

#ifdef __cplusplus
}
#endif

#endif 