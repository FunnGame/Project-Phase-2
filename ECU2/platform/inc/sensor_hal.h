#ifndef SENSOR_HAL_H
#define SENSOR_HAL_H

#include <stdint.h>

#define ENCODER_PPR             11.0f
#define GEAR_RATIO              34.0f
#define ENCODER_MODE_MULTIPLIER 4.0f
#define TOTAL_PPR               (ENCODER_PPR * GEAR_RATIO * ENCODER_MODE_MULTIPLIER)

void Sensor_Encoder_Init(void);

int16_t Sensor_Get_Left_Pulses(void);
int16_t Sensor_Get_Right_Pulses(void);

void Sensor_Reset_Left_Pulses(void);
void Sensor_Reset_Right_Pulses(void);

float Sensor_Get_Left_RPM_Current(void);
float Sensor_Get_Right_RPM_Current(void);

#endif