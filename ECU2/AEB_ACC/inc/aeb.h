#ifndef AEB_H
#define AEB_H

#include <stdint.h>

#define AEB_DISTANCE_INVALID (-1.0f)

typedef enum
{
    AEB_STATE_IDLE = 0,   
    AEB_STATE_WARNING,   
    AEB_STATE_BRAKING    
} AEB_State_t;


AEB_State_t AEB_Execute(float distance_cm,
                         uint16_t warning_distance_cm,
                         uint16_t brake_distance_cm,
                         uint8_t warning_speed);

#endif 