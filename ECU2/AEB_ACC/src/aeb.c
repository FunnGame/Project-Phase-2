#include "aeb.h"
#include "tB6612_Motor.h"


AEB_State_t AEB_Execute(float distance_cm,
                         uint16_t warning_distance_cm,
                         uint16_t brake_distance_cm,
                         uint8_t warning_speed)
{
   
    if (distance_cm == AEB_DISTANCE_INVALID)
    {
        return AEB_STATE_IDLE;
    }

    if (distance_cm <= (float)brake_distance_cm)
    {
        TB6612_Brake();
        return AEB_STATE_BRAKING;
    }

    if (distance_cm <= (float)warning_distance_cm)
    {
        TB6612_Drive(CAR_FORWARD, warning_speed, warning_speed);
        return AEB_STATE_WARNING;
    }

    return AEB_STATE_IDLE;
}