#include "acc.h"
#include "motor_encoder.h"
#include "pid_control.h"
#include "tb6612_motor.h"
#include "car_config.h"

static PIDController_t s_pid;
static float s_last_distance;
static float s_front_rpm;
static float s_target_rpm;
static float s_vehicle_rpm;

static float rpm_to_pwm(float rpm)
{
    float pwm = rpm * ACC_PWM_MAX / ACC_MAX_RPM;
    if (pwm < 0.0f) pwm = 0.0f;
    if (pwm > ACC_PWM_MAX) pwm = ACC_PWM_MAX;
    return pwm;
}

void ACC_Init(void)
{
    MotorEncoder_Init();
    TB6612_Init();

    s_pid.Kp = ACC_KP;
    s_pid.Ki = ACC_KI;
    s_pid.Kd = ACC_KD;
    s_pid.T = ACC_PID_T;
    s_pid.tau = ACC_PID_TAU;
    s_pid.limMin = ACC_MIN_RPM;
    s_pid.limMax = ACC_MAX_RPM;
    s_pid.limMinInt = ACC_limMinInt;
    s_pid.limMaxInt = ACC_limMaxInt;

    PID_Init(&s_pid);

    s_last_distance = 0.0f;
    s_front_rpm = 0.0f;
    s_target_rpm = 0.0f;
    s_vehicle_rpm = 0.0f;
}

void ACC_Run(uint16_t distance_mm, float distance_dt, uint16_t active_distance_mm, float setpoint_rpm)
{
    MotorEncoder_Process();

    s_vehicle_rpm = (MotorEncoder_GetLeftRPM() + MotorEncoder_GetRightRPM()) * 0.5f;

    if (distance_dt > 0.0f)
    {
        s_front_rpm = s_vehicle_rpm + (((float)distance_mm - s_last_distance) / distance_dt) * 60.0f / ACC_WHEEL_CIRCUMFERENCE;
        if (s_front_rpm < 0.0f) s_front_rpm = 0.0f;
    }

    s_last_distance = (float)distance_mm;

    if (distance_mm > active_distance_mm)
        s_target_rpm = setpoint_rpm;
    else if (s_front_rpm < setpoint_rpm)
        s_target_rpm = s_front_rpm;
    else
        s_target_rpm = setpoint_rpm;

    if (s_target_rpm < ACC_MIN_RPM) s_target_rpm = ACC_MIN_RPM;
    if (s_target_rpm > ACC_MAX_RPM) s_target_rpm = ACC_MAX_RPM;

    float output = PID_Compute(&s_pid, s_target_rpm, s_vehicle_rpm);
    uint8_t pwm = (uint8_t)rpm_to_pwm(output);

    if (pwm == 0u)
        TB6612_Stop();
    else
        TB6612_Drive(CAR_FORWARD, pwm, pwm);
}