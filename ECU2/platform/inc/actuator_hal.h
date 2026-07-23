#ifndef __ACTUATOR_HAL_H
#define __ACTUATOR_HAL_H

#include <stdint.h>


typedef enum {
    CAR_STOP = 0,
    CAR_FORWARD,
    CAR_BACKWARD,
    CAR_TURN_LEFT,
    CAR_TURN_RIGHT
} Car_Direction_t;


void Actuator_Car_Init(void);


void Actuator_Car_Control(Car_Direction_t dir, uint8_t speed_percent);

#endif 