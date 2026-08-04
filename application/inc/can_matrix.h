/*
 * can_matrix.h
 *
 * Author: trong
 */

#ifndef CAN_MATRIX_H
#define CAN_MATRIX_H

#include <stdint.h>
#include "rf_protocol.h"

/*=========================================================
 * Vehicle Command
 *========================================================*/

typedef struct
{
    int8_t throttle;
    int8_t steering;

    uint8_t mode;

} VehicleCommand_t;

/*=========================================================
 * API
 *========================================================*/

void CANMatrix_Init(void);

void CANMatrix_Update(void);

VehicleCommand_t CANMatrix_GetCommand(void);

#endif
