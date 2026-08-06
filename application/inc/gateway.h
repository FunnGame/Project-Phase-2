/*
 * gateway.h
 *
 * Author: trong
 */

#ifndef GATEWAY_H
#define GATEWAY_H

#include <stdint.h>
#include <stdbool.h>
#include "contracts/rf_protocol.h" /* Contract chung từ nhánh của Vinh */[cite: 9]

typedef enum {
    ECU3_STATE_INIT = 0,
    ECU3_STATE_OPERATIONAL,
    ECU3_STATE_SAFE_STOP,
    ECU3_STATE_FAULT
} ECU3_State_t;

void         Gateway_Init(void);
void         Gateway_ProcessRFFrame(const rf_control_frame_t *rf_frame);
void         Gateway_Tick1ms(void);
ECU3_State_t Gateway_GetState(void);

#endif /* GATEWAY_H */
