/*
 * drv_can.c
 *
 *      Author: trong
 */

#include "drv_can.h"

/*=========================================================
 * Local Function Prototype
 *========================================================*/

static void CAN_GPIO_Init(void);
static void CAN_Peripheral_Init(void);

/*=========================================================
 * Driver Initialization
 *========================================================*/

/*
 * Khởi tạo CAN
 */
void CAN_Init(void)
{
    CAN_GPIO_Init();

    CAN_Peripheral_Init();
}

/*=========================================================
 * GPIO Configuration
 *========================================================*/

/*
 * CAN1
 *
 * RX = PA11
 * TX = PA12
 */
static void CAN_GPIO_Init(void)
{
    /* Enable GPIOA */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    /* Enable AFIO */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    /*
     * PA11
     *
     * Floating Input
     */

    GPIOA->CRH &= ~(0x0FU << 12);
    GPIOA->CRH |=  (0x04U << 12);

    /*
     * PA12
     *
     * Alternate Function Push Pull
     * 50MHz
     */

    GPIOA->CRH &= ~(0x0FU << 16);
    GPIOA->CRH |=  (0x0BU << 16);
}

