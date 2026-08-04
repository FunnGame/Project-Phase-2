/**
 ******************************************************************************
 * @file    control_app.c
 * @brief   Differential-drive mixer implementation.
 ******************************************************************************
 */
#include "control_app.h"
#include "tb6612_motor.h"

/* Clamp a mixed wheel speed back into the motor driver's signed range. */
static int8_t clamp_speed(int16_t v)
{
    if (v >  100) return  100;
    if (v < -100) return -100;
    return (int8_t)v;
}

void ControlApp_Init(void)
{
    TB6612_Init();
    ControlApp_Stop();
}

void ControlApp_Drive(int8_t throttle, int8_t steering)
{
    /* Arcade-drive mix: throttle is the common term, steering the difference.
     * Turning right (+steering) speeds the left wheel and slows the right.
     * With zero throttle the two wheels get equal and opposite speeds, i.e.
     * the car spins in place. Widen to int16 so the sum cannot overflow int8
     * before it is clamped. */
    const int16_t left  = (int16_t)throttle + (int16_t)steering;
    const int16_t right = (int16_t)throttle - (int16_t)steering;

    TB6612_SetMotor(MOTOR_LEFT,  clamp_speed(left));
    TB6612_SetMotor(MOTOR_RIGHT, clamp_speed(right));
}

void ControlApp_Stop(void)
{
    TB6612_Stop();
}
