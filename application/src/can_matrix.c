/*
 * can_matrix.c
 *
 * Author: trong
 */

#include "can_matrix.h"

static VehicleCommand_t command;
void CANMatrix_Init(void)
{
    command.throttle = 0;
    command.steering = 0;
    command.mode = 0;
}
VehicleCommand_t CANMatrix_GetCommand(void)
{
    return command;
}
void CANMatrix_Update(void)
{
    RF_ControlPacket_t packet;

    if(!RF_IsConnected())
    {
        return;
    }

    packet = RF_GetControl();

    command.throttle = packet.throttle;
    command.steering = packet.steering;
    command.mode = packet.mode;
}
