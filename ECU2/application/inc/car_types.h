#ifndef CAR_TYPES_H_
#define CAR_TYPES_H_

#include <stdint.h>
#include "actuator_hal.h"


typedef enum {
    MODE_AUTO = 0,
    MODE_MANUAL
} SystemMode_t;

typedef enum {
    ACT_OFF = 0,
    ACT_ON
} ActuatorState_t;


typedef struct {
    Car_Direction_t direction;          
    uint8_t         speed_percent;        
    SystemMode_t    mode;               
} CarData_t;

#endif