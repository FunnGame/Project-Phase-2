///*
// * nrf24_hal.c
// *
// * Created on: 23 Jul 2026
// * Author: trong
// *
// * NRF24L01 Hardware Abstraction Layer
// * Communication ECU (Receiver)
// */
//
//#include "nrf24_hal.h"
//#include "drv_spi.h"
//
///*=========================================================
// * CE Pin Control
// * CE -> PB11
// *========================================================*/
//#define NRF24_CE_PORT      GPIOB
//#define NRF24_CE_PIN       11
//
//#define NRF24_CE_LOW()     (NRF24_CE_PORT->BRR  = (1U << NRF24_CE_PIN))
//#define NRF24_CE_HIGH()    (NRF24_CE_PORT->BSRR = (1U << NRF24_CE_PIN))
//
///*
// * 5-byte RF address.
// * This address must be identical on
// * both transmitter and receiver.
// */
//static const uint8_t defaultRfAddress[NRF_ADDR_WIDTH] =
//{
//    0x34,
//    0x43,
//    0x10,
//    0x10,
//    0x01
//};
//
///*=========================================================
// * Local Function
// *========================================================*/
//
///**
// * @brief Configure CE pin as Push-Pull Output
// */
//static void NRF24_CE_Init(void)
//{
//    /* Enable GPIOB Clock */
//    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
//
//    /*
//     * PB11
//     * MODE11 = 11 (50MHz)
//     * CNF11  = 00 (General Purpose Push Pull)
//     */
//
//    GPIOB->CRH &= ~(0x0FU << 12);
//    GPIOB->CRH |=  (0x03U << 12);
//
//    NRF24_CE_LOW();
//}
//
///*=========================================================
// * Register Access
// *========================================================*/
//
///**
// * @brief Read one register
// */
//uint8_t NRF24_ReadReg(uint8_t reg)
//{
//    uint8_t value;
//
//    SPI2_CSN_Select();
//
//    SPI2_TransmitReceive(NRF_CMD_R_REGISTER | (reg & 0x1F));
//    value = SPI2_TransmitReceive(NRF_CMD_NOP);
//
//    SPI2_CSN_Deselect();
//
//    return value;
//}
//
///**
// * @brief Write one register
// */
//void NRF24_WriteReg(uint8_t reg, uint8_t value)
//{
//    SPI2_CSN_Select();
//
//    SPI2_TransmitReceive(NRF_CMD_W_REGISTER | (reg & 0x1F));
//    SPI2_TransmitReceive(value);
//
//    SPI2_CSN_Deselect();
//}
//
///*=========================================================
// * Buffer Access
// *========================================================*/
//
///**
// * @brief Read multiple bytes
// */
//void NRF24_ReadBuffer(uint8_t command,
//                      uint8_t *buffer,
//                      uint8_t length)
//{
//    SPI2_CSN_Select();
//
//    SPI2_TransmitReceive(command);
//
//    for(uint8_t i = 0; i < length; i++)
//    {
//        buffer[i] = SPI2_TransmitReceive(NRF_CMD_NOP);
//    }
//
//    SPI2_CSN_Deselect();
//}
//
///**
// * @brief Write multiple bytes
// */
//void NRF24_WriteBuffer(uint8_t command,
//                       const uint8_t *buffer,
//                       uint8_t length)
//{
//    SPI2_CSN_Select();
//
//    SPI2_TransmitReceive(command);
//
//    for(uint8_t i = 0; i < length; i++)
//    {
//        SPI2_TransmitReceive(buffer[i]);
//    }
//
//    SPI2_CSN_Deselect();
//}
//
///*=========================================================
// * Status
// *========================================================*/
//
///**
// * @brief Get STATUS register
// *
// * STATUS is returned during every SPI command.
// * Here we simply send NOP to read it.
// */
//uint8_t NRF24_GetStatus(void)
//{
//    uint8_t status;
//
//    SPI2_CSN_Select();
//
//    status = SPI2_TransmitReceive(NRF_CMD_NOP);
//
//    SPI2_CSN_Deselect();
//
//    return status;
//}
//
///**
// * @brief Clear all interrupt flags
// */
//void NRF24_ClearIRQ(void)
//{
//    NRF24_WriteReg(
//            NRF_REG_STATUS,
//            NRF_STATUS_RX_DR |
//            NRF_STATUS_TX_DS |
//            NRF_STATUS_MAX_RT
//    );
//}
//
///*=========================================================
// * FIFO Control
// *========================================================*/
//
///**
// * @brief Flush RX FIFO
// */
//void NRF24_FlushRX(void)
//{
//    SPI2_CSN_Select();
//
//    SPI2_TransmitReceive(NRF_CMD_FLUSH_RX);
//
//    SPI2_CSN_Deselect();
//}
//
///**
// * @brief Flush TX FIFO
// */
//void NRF24_FlushTX(void)
//{
//    SPI2_CSN_Select();
//
//    SPI2_TransmitReceive(NRF_CMD_FLUSH_TX);
//
//    SPI2_CSN_Deselect();
//}
///*=========================================================
// * Driver Initialization
// *========================================================*/
//
///**
// * @brief Initialize NRF24L01 in Receiver Mode
// */
//void NRF24_Init(void)
//{
//    SPI2_Init();
//    NRF24_CE_Init();
//
//    /* Stop listening while configuring */
//    NRF24_CE_LOW();
//
//    /*-----------------------------------------------------
//     * CONFIG
//     *
//     * EN_CRC  = 1
//     * CRCO    = 1 (2-byte CRC)
//     * PWR_UP  = 1
//     * PRIM_RX = 1 (Receiver)
//     *----------------------------------------------------*/
//    NRF24_WriteReg(
//            NRF_REG_CONFIG,
//            NRF_CONFIG_EN_CRC |
//            NRF_CONFIG_CRCO |
//            NRF_CONFIG_PWR_UP |
//            NRF_CONFIG_PRIM_RX);
//    /* Wait 2ms after Power Up
//     * Datasheet requires tpd2stby ≈ 1.5ms
//     */
//    // TODO:
//    // Delay 2ms here when system delay function is available.
//
//    /* Disable Auto Acknowledge */
//    NRF24_WriteReg(
//            NRF_REG_EN_AA,
//            NRF_AUTO_ACK_DISABLE);
//
//    /* Enable Data Pipe 0 */
//    NRF24_WriteReg(
//            NRF_REG_EN_RXADDR,
//            NRF_PIPE0_ENABLE);
//
//    /* Address Width = 5 bytes */
//    NRF24_WriteReg(
//            NRF_REG_SETUP_AW,
//            NRF_ADDR_WIDTH_CFG);
//
//    /* Disable Auto Retransmit */
//    NRF24_WriteReg(
//            NRF_REG_SETUP_RETR,
//            NRF_RETR_DISABLE);
//
//    /* RF Channel (2400 + 76 = 2476 MHz) */
//    NRF24_WriteReg(
//            NRF_REG_RF_CH,
//            NRF_RF_CHANNEL);
//
//    /*
//     * RF_SETUP
//     *
//     * 1Mbps
//     * 0dBm
//     */
//    NRF24_WriteReg(
//            NRF_REG_RF_SETUP,
//            NRF_RF_SETUP_1MBPS_0DBM);
//
//    /* RX Address Pipe 0 */
//    NRF24_WriteBuffer(
//            NRF_CMD_W_REGISTER | NRF_REG_RX_ADDR_P0,
//			defaultRfAddress,
//            NRF_ADDR_WIDTH);
//
//    /* TX Address */
//    NRF24_WriteBuffer(
//            NRF_CMD_W_REGISTER | NRF_REG_TX_ADDR,
//			defaultRfAddress,
//            NRF_ADDR_WIDTH);
//
//    /* Static Payload Size */
//    NRF24_WriteReg(
//            NRF_REG_RX_PW_P0,
//            sizeof(RF_ControlPacket_t));
//
//    /* Clear pending interrupt flags */
//    NRF24_ClearIRQ();
//
//    /* Flush FIFOs */
//    NRF24_FlushRX();
//    NRF24_FlushTX();
//
//    /* Enter RX Mode */
//    NRF24_CE_HIGH();
//}
//
///*=========================================================
// * Receive
// *========================================================*/
//
///**
// * @brief Check whether new packet is available
// *
// * @retval 1 : New packet available
// * @retval 0 : No packet
// */
//uint8_t NRF24_Available(void)
//{
//    uint8_t status = NRF24_GetStatus();
//
//    if(status & NRF_STATUS_RX_DR)
//    {
//        return 1U;
//    }
//
//    return 0U;
//}
//
///**
// * @brief Receive one RF packet
// */
//void NRF24_Receive(RF_ControlPacket_t *packet)
//{
//    if(packet == NULL)
//    {
//        return;
//    }
//    if(sizeof(RF_ControlPacket_t) > NRF_MAX_PAYLOAD_SIZE)
//    {
//        return;
//    }
//
//    NRF24_ReadBuffer(
//            NRF_CMD_R_RX_PAYLOAD,
//            (uint8_t *)packet,
//            sizeof(RF_ControlPacket_t));
//
//    /* Clear RX interrupt */
//    NRF24_ClearIRQ();
//
//    /* Flush RX FIFO to remove old packet */
//    NRF24_FlushRX();
//}
