/**
 ******************************************************************************
 * @file    tb6612_motor.c
 * @brief   TB6612FNG dual H-bridge driver implementation.
 ******************************************************************************
 */
#include "tb6612_motor.h"

#include "drv_gpio.h"
#include "drv_pwm.h"
#include "car_config.h"


typedef struct {
    GPIO_TypeDef *in1_port;
    uint8_t       in1_pin;
    GPIO_TypeDef *in2_port;
    uint8_t       in2_pin;
    PWM_Channel   channel;
    bool          invert;
} Motor_Wiring;

static const Motor_Wiring s_motors[2] = {
    [MOTOR_LEFT] = {
        .in1_port = CAR_MOTOR_L_IN1_PORT, .in1_pin = CAR_MOTOR_L_IN1_PIN,
        .in2_port = CAR_MOTOR_L_IN2_PORT, .in2_pin = CAR_MOTOR_L_IN2_PIN,
        .channel  = CAR_MOTOR_L_PWM_CHANNEL,
        .invert   = (CAR_MOTOR_L_INVERT != 0),
    },
    [MOTOR_RIGHT] = {
        .in1_port = CAR_MOTOR_R_IN1_PORT, .in1_pin = CAR_MOTOR_R_IN1_PIN,
        .in2_port = CAR_MOTOR_R_IN2_PORT, .in2_pin = CAR_MOTOR_R_IN2_PIN,
        .channel  = CAR_MOTOR_R_PWM_CHANNEL,
        .invert   = (CAR_MOTOR_R_INVERT != 0),
    },
};


static void motor_set_bridge(const Motor_Wiring *m, bool in1, bool in2)
{
    DRV_GPIO_Write(m->in1_port, m->in1_pin, in1 ? GPIO_HIGH : GPIO_LOW);
    DRV_GPIO_Write(m->in2_port, m->in2_pin, in2 ? GPIO_HIGH : GPIO_LOW);
}


void TB6612_Init(void)
{
    DRV_GPIO_InitOutput(CAR_MOTOR_STBY_PORT, CAR_MOTOR_STBY_PIN, GPIO_LOW);

    for (unsigned i = 0; i < DRV_ARRAY_LEN(s_motors); ++i) {
        const Motor_Wiring *m = &s_motors[i];
        DRV_GPIO_InitOutput(m->in1_port, m->in1_pin, GPIO_LOW);
        DRV_GPIO_InitOutput(m->in2_port, m->in2_pin, GPIO_LOW);
    }

    const PWM_Config left = {
        .tim       = CAR_MOTOR_PWM_TIMER,
        .channel   = CAR_MOTOR_L_PWM_CHANNEL,
        .port      = CAR_MOTOR_L_PWM_PORT,
        .pin       = CAR_MOTOR_L_PWM_PIN,
        .frequency = CAR_MOTOR_PWM_HZ,
        .polarity  = PWM_ACTIVE_HIGH,
    };
    const PWM_Config right = {
        .tim       = CAR_MOTOR_PWM_TIMER,
        .channel   = CAR_MOTOR_R_PWM_CHANNEL,
        .port      = CAR_MOTOR_R_PWM_PORT,
        .pin       = CAR_MOTOR_R_PWM_PIN,
        .frequency = CAR_MOTOR_PWM_HZ,
        .polarity  = PWM_ACTIVE_HIGH,
    };
    (void)DRV_PWM_Init(&left);
    (void)DRV_PWM_Init(&right);

    DRV_PWM_Start(CAR_MOTOR_PWM_TIMER, CAR_MOTOR_L_PWM_CHANNEL);
    DRV_PWM_Start(CAR_MOTOR_PWM_TIMER, CAR_MOTOR_R_PWM_CHANNEL);

    TB6612_Stop();
    TB6612_SetStandby(true);
}

void TB6612_SetMotor(Motor_Side_t side, int8_t speed)
{
    if (side != MOTOR_LEFT && side != MOTOR_RIGHT) {
        return;
    }
    const Motor_Wiring *m = &s_motors[side];

    bool forward = (speed > 0);
    if (speed < -100) speed = -100;
    if (speed >  100) speed =  100;

    uint8_t magnitude = (uint8_t)((speed < 0) ? -speed : speed);

    if (m->invert) {
        forward = !forward;
    }

    if (magnitude == 0u) {
        motor_set_bridge(m, false, false);     
    } else {
        motor_set_bridge(m, forward, !forward);
    }
    DRV_PWM_SetDuty(CAR_MOTOR_PWM_TIMER, m->channel, magnitude);
}

void TB6612_Drive(Car_Direction_t dir, uint8_t left_speed, uint8_t right_speed)
{
    if (left_speed  > 100u) left_speed  = 100u;
    if (right_speed > 100u) right_speed = 100u;

    const int8_t l = (int8_t)left_speed;
    const int8_t r = (int8_t)right_speed;

    switch (dir) {
        case CAR_FORWARD:
            TB6612_SetMotor(MOTOR_LEFT,   l);
            TB6612_SetMotor(MOTOR_RIGHT,  r);
            break;

        case CAR_BACKWARD:
            TB6612_SetMotor(MOTOR_LEFT,  (int8_t)-l);
            TB6612_SetMotor(MOTOR_RIGHT, (int8_t)-r);
            break;

        case CAR_TURN_LEFT:     
            TB6612_SetMotor(MOTOR_LEFT,  (int8_t)-l);
            TB6612_SetMotor(MOTOR_RIGHT,  r);
            break;

        case CAR_TURN_RIGHT:     
            TB6612_SetMotor(MOTOR_LEFT,   l);
            TB6612_SetMotor(MOTOR_RIGHT, (int8_t)-r);
            break;

        case CAR_STOP:
        default:
            TB6612_Stop();
            break;
    }
}

void TB6612_Stop(void)
{
    for (unsigned i = 0; i < DRV_ARRAY_LEN(s_motors); ++i) {
        const Motor_Wiring *m = &s_motors[i];
        motor_set_bridge(m, false, false);
        DRV_PWM_SetDuty(CAR_MOTOR_PWM_TIMER, m->channel, 0u);
    }
}

void TB6612_Brake(void)
{
    for (unsigned i = 0; i < DRV_ARRAY_LEN(s_motors); ++i) {
        const Motor_Wiring *m = &s_motors[i];
        motor_set_bridge(m, true, true);
        DRV_PWM_SetDuty(CAR_MOTOR_PWM_TIMER, m->channel, 100u);
    }
}

void TB6612_SetStandby(bool enabled)
{
    DRV_GPIO_Write(CAR_MOTOR_STBY_PORT, CAR_MOTOR_STBY_PIN,
                   enabled ? GPIO_HIGH : GPIO_LOW);
}