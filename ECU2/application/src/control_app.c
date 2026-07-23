#include "control_app.h"
#include "actuator_hal.h"

static CarData_t g_car_data;

void ControlApp_Init(void) {
    g_car_data.direction     = CAR_STOP;
    g_car_data.speed_percent = 0;
    g_car_data.mode          = MODE_MANUAL; 

    Actuator_Car_Init();
    
    Actuator_Car_Control(g_car_data.direction, g_car_data.speed_percent);
}

void ControlApp_Set(Car_Direction_t new_dir, uint8_t speed_percent) {
    g_car_data.direction     = new_dir;
    g_car_data.speed_percent = speed_percent;
}

void ControlApp_Update(void) {
    switch (g_car_data.mode) {
        
        case MODE_MANUAL:
            Actuator_Car_Control(g_car_data.direction, g_car_data.speed_percent);
            break;

        case MODE_AUTO:
            /* ============================================================
             * CH? TUONG LAI:
             * 1. Ð?c c?m bi?n (Encoder, Siêu âm...)
             * 2. Tính toán PID / ACC ra output_pwm
             * 3. Actuator_Car_Control(g_car_data.direction, output_pwm);
             * ============================================================ */
            break;
        default:
            Actuator_Car_Control(CAR_STOP, 0);
            break;
    }
}

CarData_t ControlApp_GetData(void) {
    return g_car_data;
}