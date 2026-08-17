#include "drv_uart.h"
#include "drv_timer.h"
#include "hal_mpu.h"
#include "hal_vl53.h"
#include "../inc/drv_i2c.h" 
#include <stdio.h>

int main(void)
{
    char uart_buf[100];
    int16_t ax, ay, az;
    uint16_t distance;

    DRV_UART_Init(UART_1, 115200);
    DRV_TIMER_Init(TIMER_4);
    DRV_UART_SendString(UART_1, "\r\n=== SENSOR FUSION BOOTING ===\r\n");

    DRV_I2C_Init(I2C_1, I2C_SPEED_100KHZ); 
    DRV_DelayMs(100U); 

    if(HAL_MPU_Init() == STATUS_OK) {
        DRV_UART_SendString(UART_1, ">>> MPU6500 INIT OK <<<\r\n");
    } else {
        DRV_UART_SendString(UART_1, ">>> MPU6500 INIT FAILED <<<\r\n");
    }

    if(HAL_VL53_Init() == STATUS_OK) {
        DRV_UART_SendString(UART_1, ">>> VL53L1X INIT OK <<<\r\n");
    } else {
        DRV_UART_SendString(UART_1, ">>> VL53L1X INIT FAILED <<<\r\n");
    }

    DRV_UART_SendString(UART_1, "--- SYSTEM READY - RUNNING ---\r\n");

    while(1)
    {
      
        if(HAL_MPU_ReadAccel(&ax, &ay, &az) == STATUS_OK) {
            sprintf(uart_buf, "MPU [X:%6d | Y:%6d | Z:%6d]  ||  ", ax, ay, az);
            DRV_UART_SendString(UART_1, uart_buf);
        }

        DRV_DelayMs(5U); 

        if(HAL_VL53_ReadDistance(&distance) == STATUS_OK) {
            sprintf(uart_buf, "TOF [Dist: %4d mm]\r\n", distance);
            DRV_UART_SendString(UART_1, uart_buf);
        }

        DRV_DelayMs(50U); 
    }
}