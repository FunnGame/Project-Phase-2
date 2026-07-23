#include "actuator_hal.h"
#include "drv_pwm.h"
#include "drv_gpio.h"

// Tien lui banh phai
#define MOTOR_L_IN1_PORT        PA
#define MOTOR_L_IN1_PIN         0
#define MOTOR_L_IN2_PORT        PA
#define MOTOR_L_IN2_PIN         1

// Tien lui banh phai
#define MOTOR_R_IN1_PORT        PA
#define MOTOR_R_IN1_PIN         2
#define MOTOR_R_IN2_PORT        PA
#define MOTOR_R_IN2_PIN         3

// Khoi dong chip
#define MOTOR_STBY_PORT         PA
#define MOTOR_STBY_PIN          4


#define MOTOR_PWM_TIM           PWM_TIM3
#define MOTOR_L_PWM_PORT        PA
#define MOTOR_L_PWM_PIN         6
#define MOTOR_L_PWM_CH          PWM_CH1

#define MOTOR_R_PWM_PORT        PA
#define MOTOR_R_PWM_PIN         7
#define MOTOR_R_PWM_CH          PWM_CH2

#define MOTOR_PWM_FREQ_HZ       10000 

static void Set_Motor_Left_Dir(uint8_t state) {
    if (state == 1) {
        GPIO_Write(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, 1);
        GPIO_Write(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, 0);
    } else if (state == 2) {
        GPIO_Write(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, 0);
        GPIO_Write(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, 1);
    } else {
        GPIO_Write(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, 0);
        GPIO_Write(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, 0);
    }
}

static void Set_Motor_Right_Dir(uint8_t state) {
    if (state == 1) {
        GPIO_Write(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, 1);
        GPIO_Write(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, 0);
    } else if (state == 2) {
        GPIO_Write(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, 0);
        GPIO_Write(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, 1);
    } else {
        GPIO_Write(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, 0);
        GPIO_Write(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, 0);
    }
}

void Actuator_Car_Init(void) {
    GPIO_Init(MOTOR_L_IN1_PORT, MOTOR_L_IN1_PIN, OUT50, O_GP_PP);
    GPIO_Init(MOTOR_L_IN2_PORT, MOTOR_L_IN2_PIN, OUT50, O_GP_PP);
    GPIO_Init(MOTOR_R_IN1_PORT, MOTOR_R_IN1_PIN, OUT50, O_GP_PP);
    GPIO_Init(MOTOR_R_IN2_PORT, MOTOR_R_IN2_PIN, OUT50, O_GP_PP);
    GPIO_Init(MOTOR_STBY_PORT,  MOTOR_STBY_PIN,  OUT50, O_GP_PP);

    GPIO_Write(MOTOR_STBY_PORT, MOTOR_STBY_PIN, 1);

    DRV_PWM_Init(MOTOR_PWM_TIM, MOTOR_L_PWM_PORT, MOTOR_L_PWM_PIN, MOTOR_L_PWM_CH, MOTOR_PWM_FREQ_HZ);
    DRV_PWM_Init(MOTOR_PWM_TIM, MOTOR_R_PWM_PORT, MOTOR_R_PWM_PIN, MOTOR_R_PWM_CH, MOTOR_PWM_FREQ_HZ);

    DRV_PWM_Start(MOTOR_PWM_TIM, MOTOR_L_PWM_CH);
    DRV_PWM_Start(MOTOR_PWM_TIM, MOTOR_R_PWM_CH);

    Actuator_Car_Control(CAR_STOP, 0);
}

void Actuator_Car_Control(Car_Direction_t dir, uint8_t speed_percent) {
    if (speed_percent > 100) {
        speed_percent = 100;
    }

    uint16_t target_duty = speed_percent;

    switch (dir) {
        case CAR_FORWARD:
            Set_Motor_Left_Dir(1);
            Set_Motor_Right_Dir(1);
            DRV_PWM_SetDuty(MOTOR_PWM_TIM, MOTOR_L_PWM_CH, target_duty);
            DRV_PWM_SetDuty(MOTOR_PWM_TIM, MOTOR_R_PWM_CH, target_duty);
            break;

        case CAR_BACKWARD:
            Set_Motor_Left_Dir(2);
            Set_Motor_Right_Dir(2);
            DRV_PWM_SetDuty(MOTOR_PWM_TIM, MOTOR_L_PWM_CH, target_duty);
            DRV_PWM_SetDuty(MOTOR_PWM_TIM, MOTOR_R_PWM_CH, target_duty);
            break;

        case CAR_TURN_LEFT:
            Set_Motor_Left_Dir(2);
            Set_Motor_Right_Dir(1);
            DRV_PWM_SetDuty(MOTOR_PWM_TIM, MOTOR_L_PWM_CH, target_duty);
            DRV_PWM_SetDuty(MOTOR_PWM_TIM, MOTOR_R_PWM_CH, target_duty);
            break;

        case CAR_TURN_RIGHT:
            Set_Motor_Left_Dir(1);
            Set_Motor_Right_Dir(2);
            DRV_PWM_SetDuty(MOTOR_PWM_TIM, MOTOR_L_PWM_CH, target_duty);
            DRV_PWM_SetDuty(MOTOR_PWM_TIM, MOTOR_R_PWM_CH, target_duty);
            break;

        case CAR_STOP:
        default:
            Set_Motor_Left_Dir(0);
            Set_Motor_Right_Dir(0);
            DRV_PWM_SetDuty(MOTOR_PWM_TIM, MOTOR_L_PWM_CH, 0);
            DRV_PWM_SetDuty(MOTOR_PWM_TIM, MOTOR_R_PWM_CH, 0);
            break;
    }
}