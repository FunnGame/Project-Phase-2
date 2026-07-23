#include "stm32f10x.h"
#include "actuator_hal.h"

int main(void) {
    SystemInit(); 

    
    Actuator_Car_Init();

    
    Actuator_Car_Control(CAR_TURN_RIGHT, 5);

    

    while (1) {
        
    }
}