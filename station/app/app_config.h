/**
 ******************************************************************************
 * @file    app_config.h
 * @brief   Central board configuration for the station receiver.
 *
 * Edit pins, ports, peripheral instances and RF-link parameters here
 *
 * Default target: Nucleo-F446RE.
 *   - USART2 (PA2/PA3) is hard-wired to the ST-Link virtual COM port.
 *   - LD2 user LED is on PA5.
 *   - SPI2 carries the nRF24.
 *    
 ******************************************************************************
 */
#ifndef STATION_APP_CONFIG_H
#define STATION_APP_CONFIG_H

#include "gpio.h"          /* GPIO_AFx, GPIO port symbols */
#include "spi.h"           /* SPI_BAUD_* */
#include "rf_protocol.h"   /* RF_CONTROL_FRAME_SIZE */
#include "nrf24.h"         /* NRF24_DR_*, NRF24_PWR_* */

/* ===== System clock ======================================================= */
/* HSE frequency in Hz, or 0 to run 180 MHz from the internal HSI.
 * A Nucleo-F446RE fed by the 8 MHz ST-Link MCO would use 8000000. */
#define APP_HSE_HZ              0u

/* ===== Status LED (LD2) ==================================================== */
#define APP_LED_PORT            GPIOA
#define APP_LED_PIN             5u

/* ===== Command UART (laptop link via ST-Link VCP) ========================= */
#define APP_UART                USART2
#define APP_UART_BAUD           115200u
#define APP_UART_GPIO           GPIOA
#define APP_UART_TX_PIN         2u
#define APP_UART_RX_PIN         3u
#define APP_UART_AF             GPIO_AF7

/* ===== nRF24 SPI bus ======================================================= */
#define APP_NRF_SPI             SPI2
#define APP_NRF_SPI_BAUD        SPI_BAUD_DIV8    /* ~5.6 MHz @ 45 MHz PCLK1 */

#define APP_NRF_SCK_PORT        GPIOB
#define APP_NRF_SCK_PIN         13u
#define APP_NRF_SCK_AF          GPIO_AF5

#define APP_NRF_MOSI_PORT       GPIOB
#define APP_NRF_MOSI_PIN        15u
#define APP_NRF_MOSI_AF         GPIO_AF5

#define APP_NRF_MISO_PORT       GPIOB
#define APP_NRF_MISO_PIN        14u
#define APP_NRF_MISO_AF         GPIO_AF5

/* ===== nRF24 control lines ================================================= */
#define APP_NRF_CSN_PORT        GPIOB
#define APP_NRF_CSN_PIN         6u
#define APP_NRF_CE_PORT         GPIOB
#define APP_NRF_CE_PIN          12u
#define APP_NRF_IRQ_PORT        GPIOC   /* reserved for the future RX path */
#define APP_NRF_IRQ_PIN         7u

/* ===== RF link parameters (must match the car) ============================ */
#define APP_RF_CHANNEL          76u
#define APP_RF_ADDRESS          { 0xE7u, 0xE7u, 0xE7u, 0xE7u, 0xE7u }
#define APP_RF_PAYLOAD          RF_CONTROL_FRAME_SIZE   /* 7 bytes */
#define APP_RF_DATARATE         NRF24_DR_1MBPS
#define APP_RF_POWER            NRF24_PWR_0DBM
#define APP_RF_AUTO_ACK         true

/* ===== Behaviour =========================================================== */
/* Treat the laptop link as lost if no valid frame arrives within this window. */
#define APP_FAILSAFE_MS         150u


#endif /* STATION_APP_CONFIG_H */
