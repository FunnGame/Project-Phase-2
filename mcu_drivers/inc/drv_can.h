/*
 * drv_can.h
 *
 * Author: trong
 */

#ifndef DRV_CAN_H
#define DRV_CAN_H

#include "stm32f1xx.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t std_id;
    uint8_t  dlc;
    uint8_t  data[8];
} CAN_TxHeader_t;

typedef struct {
    uint32_t std_id;
    uint8_t  dlc;
    uint8_t  data[8];
} CAN_RxHeader_t;

void     DRV_CAN_Init(void);
bool     DRV_CAN_Transmit(const CAN_TxHeader_t *header);
bool     DRV_CAN_Receive(CAN_RxHeader_t *header);

#endif /* DRV_CAN_H */
