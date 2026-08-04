#ifndef PID_CONTROL_H
#define PID_CONTROL_H

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    
    float T;

   
    float tau;

   
    float limMin;
    float limMax;

    
    float limMinInt;
    float limMaxInt;

    
    float integrator;
    float prevError;
    float differentiator;
    float prevMeasurement;

    
    float out;

} PIDController_t;


void PID_Init(PIDController_t *pid);


float PID_Compute(PIDController_t *pid,
                  float setpoint,
                  float measurement);


void PID_Reset(PIDController_t *pid);

#endif