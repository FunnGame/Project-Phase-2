///*
// * rf_protocol.c
// *
// *  Author: trong
// */
//
//#include "rf_protocol.h"
//
//static RF_ControlPacket_t currentPacket;
//
//static uint8_t connected = 0;
//
//static uint8_t lastSequence = 0;
//
//static uint8_t RF_CheckSequence(const RF_ControlPacket_t *packet)
//{
//    if(packet->sequence == lastSequence)
//    {
//        return 0;
//    }
//
//    lastSequence = packet->sequence;
//
//    return 1;
//}
//
//static uint8_t RF_CalculateCRC(const RF_ControlPacket_t *packet)
//{
//    return (uint8_t)(
//            packet->throttle ^
//            packet->steering ^
//            packet->mode ^
//            packet->sequence);
//}
//
//static uint8_t RF_CheckCRC(const RF_ControlPacket_t *packet)
//{
//    return (RF_CalculateCRC(packet) == packet->crc);
//}
//
//void RF_Init(void)
//{
//    connected = 0;
//}
//
//RF_ControlPacket_t RF_GetControl(void)
//{
//    return currentPacket;
//}
//
//uint8_t RF_IsConnected(void)
//{
//    return connected;
//}
//
//void RF_Update(void)
//{
//    RF_ControlPacket_t packet;
//
//    if(!NRF24_Available())
//    {
//        return;
//    }
//
//    NRF24_Receive(&packet);
//
//    if(!RF_CheckCRC(&packet))
//    {
//        return;
//    }
//
//    if(!RF_CheckSequence(&packet))
//    {
//        return;
//    }
//
//    currentPacket = packet;
//
//    connected = 1;
//}
