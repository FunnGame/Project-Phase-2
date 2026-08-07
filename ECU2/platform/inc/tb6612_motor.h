#ifndef TB6612_MOTOR_H_
#define TB6612_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
} Motor_Side_t;

typedef enum {
    CAR_STOP = 0,    
    CAR_FORWARD,     
    CAR_BACKWARD,    
    CAR_TURN_LEFT,   
    CAR_TURN_RIGHT,  
} Car_Direction_t;

void TB6612_Init(void);

void TB6612_SetMotor(Motor_Side_t side, int8_t speed);

void TB6612_Drive(Car_Direction_t dir, uint8_t left_speed, uint8_t right_speed);

void TB6612_Stop(void);

void TB6612_Brake(void);

void TB6612_SetStandby(bool enabled);

#ifdef __cplusplus
}
#endif

#endif