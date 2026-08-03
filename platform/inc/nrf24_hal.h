/*
 * nrf24_hal.h
 *
 * Created on: 23 Jul 2026
 * Author: trong
 *
 * NRF24L01 Hardware Abstraction Layer
 * Communication ECU - Receiver (RX Mode)
 */

#ifndef INC_NRF24_HAL_H_
#define INC_NRF24_HAL_H_

#include <stdint.h>

/*=========================================================
 * NRF24 Commands
 *========================================================*/
#define NRF_CMD_R_REGISTER        0x00
#define NRF_CMD_W_REGISTER        0x20
#define NRF_CMD_R_RX_PAYLOAD      0x61
#define NRF_CMD_W_TX_PAYLOAD      0xA0
#define NRF_CMD_FLUSH_TX          0xE1
#define NRF_CMD_FLUSH_RX          0xE2
#define NRF_CMD_NOP               0xFF

/*=========================================================
 * NRF24 Registers
 *========================================================*/
#define NRF_REG_CONFIG            0x00
#define NRF_REG_EN_AA             0x01
#define NRF_REG_EN_RXADDR         0x02
#define NRF_REG_SETUP_AW          0x03
#define NRF_REG_SETUP_RETR        0x04
#define NRF_REG_RF_CH             0x05
#define NRF_REG_RF_SETUP          0x06
#define NRF_REG_STATUS            0x07
#define NRF_REG_OBSERVE_TX        0x08
#define NRF_REG_RPD               0x09

#define NRF_REG_RX_ADDR_P0        0x0A
#define NRF_REG_RX_ADDR_P1        0x0B
#define NRF_REG_TX_ADDR           0x10

#define NRF_REG_RX_PW_P0          0x11

#define NRF_ADDR_WIDTH 5U

/*=========================================================
 * RF Configuration
 *========================================================*/
#define NRF_RF_CHANNEL            76U
#define NRF_RF_SETUP_1MBPS_0DBM   0x06U

#define NRF_AUTO_ACK_DISABLE      0x00U
#define NRF_PIPE0_ENABLE          0x01U
#define NRF_ADDR_WIDTH_CFG        0x03U
#define NRF_RETR_DISABLE          0x00U

/*=========================================================
 * STATUS Register Flags
 *========================================================*/
#define NRF_STATUS_RX_DR          (1U << 6)
#define NRF_STATUS_TX_DS          (1U << 5)
#define NRF_STATUS_MAX_RT         (1U << 4)

/*=========================================================
 * CONFIG Register Bits
 *========================================================*/
#define NRF_CONFIG_PRIM_RX        (1U << 0)
#define NRF_CONFIG_PWR_UP         (1U << 1)
#define NRF_CONFIG_CRCO           (1U << 2)
#define NRF_CONFIG_EN_CRC         (1U << 3)

/* EN_RXADDR Register */
#define NRF_PIPE0            (1U << 0)
#define NRF_PIPE1            (1U << 1)

/* EN_AA Register */
#define NRF_AUTO_ACK_PIPE0   (1U << 0)
#define NRF_AUTO_ACK_PIPE1   (1U << 1)

/* Payload Size */
#define NRF_MAX_PAYLOAD_SIZE     32U

/*=========================================================
 * RF Packet Definition
 *========================================================*/
typedef struct
{
    int8_t throttle;
    int8_t steering;
    uint8_t mode;

    uint8_t sequence;

    uint8_t crc;

} RF_ControlPacket_t;

/*=========================================================
 * Driver API
 *========================================================*/

/* Initialization */
void NRF24_Init(void);

/* Register Access */
uint8_t NRF24_ReadReg(uint8_t reg);
void NRF24_WriteReg(uint8_t reg, uint8_t value);

/* Buffer Access */
void NRF24_ReadBuffer(uint8_t command,
                      uint8_t *buffer,
                      uint8_t length);
void NRF24_WriteBuffer(uint8_t command,
                       const uint8_t *buffer,
                       uint8_t length);

/* Status */
uint8_t NRF24_GetStatus(void);
uint8_t NRF24_Available(void);

/* Payload */
void NRF24_Receive(RF_ControlPacket_t *packet);

/* FIFO */
void NRF24_FlushRX(void);
void NRF24_FlushTX(void);

/* Interrupt */
void NRF24_ClearIRQ(void);

#endif /* INC_NRF24_HAL_H_ */
