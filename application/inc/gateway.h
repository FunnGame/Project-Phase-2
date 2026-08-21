/*
 * gateway.h
 *
 * Cập nhật: Tích hợp logic Uplink Telemetry
 */

#ifndef GATEWAY_H
#define GATEWAY_H

#include <stdint.h>
#include <stdbool.h>
#include "contracts/rf_protocol.h"

typedef enum {
    ECU3_STATE_INIT = 0,
    ECU3_STATE_OPERATIONAL,
    ECU3_STATE_SAFE_STOP,
    ECU3_STATE_FAULT
} ECU3_State_t;

void         Gateway_Init(void);
void         Gateway_ProcessRFFrame(const rf_control_frame_t *rf_frame);
/* Hàm mới: Hứng dữ liệu UPLINK từ mạng CAN */
void         Gateway_ProcessCANFrame(uint32_t can_id, const uint8_t *payload);
void         Gateway_Tick1ms(void);
ECU3_State_t Gateway_GetState(void);

#endif /* GATEWAY_H */
