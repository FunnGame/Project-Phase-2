#include "stm32f10x.h"
#include "control_app.h"
#include "sensor_hal.h"
#include <stdio.h>
#include <string.h>

static void UART3_Init(uint32_t baud) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    GPIOB->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
    GPIOB->CRH |= GPIO_CRH_MODE10_1 | GPIO_CRH_MODE10_0 | GPIO_CRH_CNF10_1;
    GPIOB->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11);
    GPIOB->CRH |= GPIO_CRH_CNF11_0;
    uint32_t pclk1 = 36000000UL;
    USART3->BRR = (pclk1 + (baud / 2U)) / baud;
    USART3->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static void UART3_SendChar(char c) {
    while (!(USART3->SR & USART_SR_TXE)) { }
    USART3->DR = (uint8_t)c;
}

static void UART3_SendString(const char *s) {
    while (*s) {
        UART3_SendChar(*s++);
    }
}

static void Print_RPM(void) {
    char buf[64];
    float left_rpm  = Sensor_Get_Left_RPM_Current();
    float right_rpm = Sensor_Get_Right_RPM_Current();
    sprintf(buf, "L_RPM=%.2f  R_RPM=%.2f\r\n", left_rpm, right_rpm);
    UART3_SendString(buf);
}

void Delay_ms(uint32_t ms) {
    uint32_t i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 0x2AFF; j++) {
            __NOP();
        }
    }
}

int main(void) {
    SystemInit();
    UART3_Init(115200);
    ControlApp_Init();   /* da tu goi Sensor_Encoder_Init() + Actuator_Car_Init() ben trong */

    UART3_SendString("== Bat dau doc encoder qua PuTTY ==\r\n");

    uint32_t elapsed_since_print = 0;

    while (1) {
        ControlApp_Update();      /* PID chay dung moi 20ms, khop voi CONTROL_LOOP_PERIOD_S */
        Delay_ms(20);

        elapsed_since_print += 20;
        if (elapsed_since_print >= 200) {   /* in RPM moi 200ms, khong lam sai timing PID */
            elapsed_since_print = 0;
            Print_RPM();
        }
    }
}