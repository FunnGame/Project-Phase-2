#include "sensor_hal.h"
#include "drv_timer.h"

static volatile int16_t left_pulses = 0;
static volatile int16_t right_pulses = 0;

static volatile float left_rpm = 0.0f;
static volatile float right_rpm = 0.0f;

static uint16_t last_left_cnt = 0;
static uint16_t last_right_cnt = 0;

static void Sensor_Calculate_Task(void) {
    uint16_t cur_left = Timer_Get_Counter(T1);
    uint16_t cur_right = Timer_Get_Counter(T4);

    left_pulses = (int16_t)(cur_left - last_left_cnt);
    right_pulses = (int16_t)(cur_right - last_right_cnt);

    last_left_cnt = cur_left;
    last_right_cnt = cur_right;

    left_rpm = ((float)left_pulses / TOTAL_PPR) * 6000.0f;
    right_rpm = ((float)right_pulses / TOTAL_PPR) * 6000.0f;
}

void Sensor_Encoder_Init(void) {
    Timer_Init();
    
    last_left_cnt = Timer_Get_Counter(T1);
    last_right_cnt = Timer_Get_Counter(T4);

    Timer2_Interrupt_Init(Sensor_Calculate_Task);
}

int16_t Sensor_Get_Left_Pulses(void) { return left_pulses; }
int16_t Sensor_Get_Right_Pulses(void) { return right_pulses; }

void Sensor_Reset_Left_Pulses(void) { left_pulses = 0; }
void Sensor_Reset_Right_Pulses(void) { right_pulses = 0; }

float Sensor_Get_Left_RPM_Current(void) { return left_rpm; }
float Sensor_Get_Right_RPM_Current(void) { return right_rpm; }