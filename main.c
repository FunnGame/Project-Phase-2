#include "app_main.h"
#include "drv_uart.h"
#include "drv_timer.h" 

int main(void)
{
    Status_t status;

    DRV_UART_Init(UART_1, 115200);
    DRV_UART_SendString(UART_1, "===============\r\n");
    DRV_UART_SendString(UART_1, "MAIN START\r\n");

    DRV_TIMER_Init(TIMER_4);
    DRV_UART_SendString(UART_1, "TIMER 4 INIT OK\r\n");

    status = App_Init();

    if(status != STATUS_OK)
    {
        DRV_UART_SendString(UART_1, "APP INIT ERROR\r\n");
        while(1){} 
    }

    DRV_UART_SendString(UART_1, "APP READY\r\n");

    while(1)
    {
        App_Run();
    }
}