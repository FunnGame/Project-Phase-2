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
 *   - SPI3 carries the ILI9341 TFT.
 *
 * The two SPI devices are deliberately on separate instances: a full-screen
 * repaint is ~150 KB of blocking pixel traffic, and sharing a bus would put
 * that in front of the radio's failsafe deadline.
 ******************************************************************************
 */
#ifndef STATION_APP_CONFIG_H
#define STATION_APP_CONFIG_H

#include "gpio.h"          /* GPIO_AFx, GPIO port symbols */
#include "spi.h"           /* SPI_BAUD_* */
#include "rf_protocol.h"   /* RF_CONTROL_FRAME_SIZE */
#include "nrf24.h"         /* NRF24_DR_*, NRF24_PWR_* */
#include "ili9341.h"       /* ILI9341_ROT_* */

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

/* ===== ILI9341 TFT SPI bus ================================================= */
#define APP_TFT_SPI             SPI3
/* ~2.8 MHz @ 45 MHz PCLK1. The ILI9341's minimum serial WRITE cycle is 100 ns
 * (10 MHz); DIV8 (5.6 MHz) is the fastest in-spec option and DIV2 (22.5 MHz)
 * is well outside it. Start slow on jumper wires, then raise one step at a
 * time - the failure mode of too-fast SPI is a blank or garbled panel, not an
 * error, because nothing reads back. */
#define APP_TFT_SPI_BAUD        SPI_BAUD_DIV16

#define APP_TFT_SCK_PORT        GPIOC
#define APP_TFT_SCK_PIN         10u
#define APP_TFT_SCK_AF          GPIO_AF6

#define APP_TFT_MOSI_PORT       GPIOC
#define APP_TFT_MOSI_PIN        12u
#define APP_TFT_MOSI_AF         GPIO_AF6

/* MISO (PC11) is left unconfigured: the panel is driven write-only, and many
 * breakout boards do not wire SDO at all. Nothing reads back from the TFT. */

/* ===== ILI9341 control lines =============================================== */
/* Plain push-pull outputs, configured by ili9341_init() itself. D/C is the
 * out-of-band bit that says whether a byte on MOSI is an opcode or data. */
#define APP_TFT_CS_PORT         GPIOC
#define APP_TFT_CS_PIN          8u
#define APP_TFT_DC_PORT         GPIOC
#define APP_TFT_DC_PIN          9u
#define APP_TFT_RST_PORT        GPIOB   /* NULL to fall back to SWRESET */
#define APP_TFT_RST_PIN         0u

/* Landscape: the dashboard is wider than it is tall, and speed/range pair
 * naturally left-right. */
#define APP_TFT_ROTATION        ILI9341_ROT_270

/* ===== RF link parameters (must match the car) ============================ */
#define APP_RF_CHANNEL          76u
#define APP_RF_ADDRESS          { 0xE7u, 0xE7u, 0xE7u, 0xE7u, 0xE7u }
#define APP_RF_PAYLOAD          RF_CONTROL_FRAME_SIZE   /* 7 bytes */
#define APP_RF_DATARATE         NRF24_DR_1MBPS
#define APP_RF_POWER            NRF24_PWR_0DBM
/* Telemetry from the car rides back on the auto-ACK of every control frame.
 * MUST match CAR_RF_ACK_PAYLOAD in car/nodes/gateway/gateway_config.h - if only
 * one end enables it, returned data jams the transmitter's RX FIFO. */
#define APP_RF_ACK_PAYLOAD      true

/* Telemetry counts as stale after this long without a valid frame. */
#define APP_TELEM_TIMEOUT_MS    500u

#define APP_RF_AUTO_ACK         true

/* ===== Behaviour =========================================================== */
/* Treat the laptop link as lost if no valid frame arrives within this window. */
#define APP_FAILSAFE_MS         150u


#endif /* STATION_APP_CONFIG_H */
