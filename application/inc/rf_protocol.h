/*
 * rf_protocol.h
 *
 * Author: trong
 */

#ifndef RF_PROTOCOL_H
#define RF_PROTOCOL_H

#include <stdint.h>
#include "nrf24_hal.h"

/* Initialize protocol layer */
void RF_Init(void);

/* Update RF state */
void RF_Update(void);

/* Check communication status */
uint8_t RF_IsConnected(void);

/* Get latest valid control packet */
RF_ControlPacket_t RF_GetControl(void);

#endif
