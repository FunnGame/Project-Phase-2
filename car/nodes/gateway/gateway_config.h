/**
 ******************************************************************************
 * @file    gateway_config.h
 * @brief   Board configuration for the GATEWAY node (STM32F103C8, node 1).
 *
 * Edit pins, ports, peripheral instances and RF-link parameters HERE — main.c
 * hard-codes nothing, so re-wiring never means hunting through the code.
 *
 * This node bridges the 2.4 GHz operator link to the CAN bus. It owns the
 * nRF24 and nothing else: no motors, no encoders, no I2C. Its counterpart is
 * car/nodes/vehicle_control/vehicle_config.h.
 *
 * IMPORTANT — the RF section must match station/app/app_config.h exactly, or
 * the two radios will not hear each other.
 ******************************************************************************
 */
#ifndef GATEWAY_CONFIG_H_
#define GATEWAY_CONFIG_H_

#include "drv_common.h"
#include "drv_gpio.h"
#include "drv_spi.h"
#include "drv_can.h"
#include "nrf24.h"
#include "rf_protocol.h"
#include "adas.h"         /* cycle times come from the DBC, not from here */

/* ===== System clock ======================================================= */
/* HSE crystal frequency in Hz. */
#define CAR_HSE_HZ              8000000u

/* 1 ms tick priority. This node uses SysTick, leaving every TIMx free. */
#define CAR_TICK_IRQ_PRIORITY   1u

/* ===== CAN vehicle bus ====================================================
 * drv_can fixes the pins at PB8 (RX) / PB9 (TX) and the rate at 500 kbit/s.
 * Message layout is contracts/adas.dbc; do not hand-code identifiers.
 *
 * PIN NOTE: PB8/PB9 are also I2C1's remap pins and TIM4 CH3/CH4. This node
 * uses neither, but check here before assigning anything new.
 */
#define CAR_NODE_ID_GATEWAY     1u        /* NodeAddress GW in adas.dbc       */

/* CAN_MODE_LOOPBACK drives the wire but needs no ACK and cannot bus-off - use
 * it when bringing this node up alone. CAN_MODE_NORMAL for real operation. */
#define CAR_CAN_MODE            CAN_MODE_NORMAL

/* Transmit periods are taken from the GENERATED header, not duplicated here:
 * they are a property of the contract, and a copy in each node's config is a
 * copy that can drift from the DBC. Change GenMsgCycleTime in adas.dbc and
 * re-run contracts/generate.sh. */
#define CAR_CAN_CMD_PERIOD_MS   ADAS_GATEWAY_DRIVER_CMD_CYCLE_TIME_MS   /* 0x200 */
#define CAR_CAN_HEARTBEAT_MS    ADAS_GW_HEARTBEAT_CYCLE_TIME_MS         /* 0x700 */

/* ===== Status LED ========================================================= */
#define CAR_LED_STATUS_PORT     GPIOC
#define CAR_LED_STATUS_PIN      13u
#define CAR_LED_ACTIVE_LOW      1

/* ===== nRF24 SPI bus ======================================================
 * SPI2 on the F103: PB13 SCK, PB14 MISO, PB15 MOSI. */
#define CAR_NRF_SPI             SPI2
#define CAR_NRF_SPI_PORT        GPIOB
#define CAR_NRF_SCK_PIN         13u
#define CAR_NRF_MISO_PIN        14u
#define CAR_NRF_MOSI_PIN        15u
#define CAR_NRF_SPI_BAUD        SPI_BAUD_DIV8   /* 36 MHz PCLK1 / 8 = 4.5 MHz */

/* ===== nRF24 control lines ================================================ */
#define CAR_NRF_CSN_PORT        GPIOB
#define CAR_NRF_CSN_PIN         12u
#define CAR_NRF_CE_PORT         GPIOB
#define CAR_NRF_CE_PIN          10u
#define CAR_NRF_IRQ_PORT        GPIOB
#define CAR_NRF_IRQ_PIN         11u

/* ===== RF link parameters — MUST MATCH THE STATION ======================== */
#define CAR_RF_CHANNEL          76u
#define CAR_RF_ADDRESS          { 0xE7u, 0xE7u, 0xE7u, 0xE7u, 0xE7u }
#define CAR_RF_PAYLOAD          RF_CONTROL_FRAME_SIZE   /* 7 bytes */
#define CAR_RF_DATARATE         NRF24_DR_1MBPS
#define CAR_RF_POWER            NRF24_PWR_0DBM
#define CAR_RF_AUTO_ACK         true

/* Telemetry rides back on the auto-ACK of every control frame, so it costs no
 * extra airtime and the car never has to become a transmitter. MUST match
 * STATION_RF_ACK_PAYLOAD in station/app/app_config.h. */
#define CAR_RF_ACK_PAYLOAD      true

/* ===== Behaviour ==========================================================
 * No valid RF frame for this long => publish LinkOk=0 with the command fields
 * zeroed. The vehicle node applies its own, shorter CAN timeout on top. */
#define CAR_FAILSAFE_MS         150u

#endif /* GATEWAY_CONFIG_H_ */
