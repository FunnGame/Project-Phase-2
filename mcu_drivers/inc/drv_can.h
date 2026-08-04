/*
 * drv_can.h
 *
 *      Author: trong
 */

#ifndef DRV_CAN_H
#define DRV_CAN_H

#include <stdint.h>
#include "stm32f10x.h"

/*=========================================================
 * CAN Identifier
 *========================================================*/
#define CAN_ID_MOTOR_CONTROL      0x101U

/*=========================================================
 * CAN Message Structure
 *========================================================*/
typedef struct
{
    uint32_t id;

    uint8_t length;

    uint8_t data[8];

} CAN_Message_t;

/*=========================================================
 * Driver API
 *========================================================*/

/* Khởi tạo CAN */
void CAN_Init(void);

/* Gửi một frame CAN */
uint8_t CAN_Send(const CAN_Message_t *message);

#endif
