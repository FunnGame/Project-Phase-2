/*
 * nrf24_hal.h
 *
 *  Created on: 23 Jul 2026
 *      Author: trong
 * Chế độ Nhận RX
 */

#ifndef INC_NRF24_HAL_H_
#define INC_NRF24_HAL_H_

#include <stdint.h>

// Struct gói tin lệnh nhận từ trạm
typedef struct {
    int8_t throttle; // Tốc độ (-100 đến 100)
    int8_t steering; // Hướng rẽ (-100 đến 100)
    uint8_t mode;    // Chế độ (0: Remote, 1: Auto)
} RF_ControlPacket_t;

void NRF24_Init(void);
uint8_t NRF24_Available(void);
void NRF24_Receive(RF_ControlPacket_t *packet);

#endif /* INC_NRF24_HAL_H_ */
