/**
 ******************************************************************************
 * @file    main.c
 * @brief   Station bridge: laptop control frames (UART) -> car (nRF24).
 *
 * Runs on the Nucleo-F446RE. The laptop's control app streams 7-byte control
 * frames (contracts/rf_protocol.h) over the ST-Link virtual COM port (USART2).
 * This app:
 *
 *   1. brings up clocks, SysTick, USART2, SPI1 + the nRF24, and LD2,
 *   2. feeds every received byte into the shared streaming parser,
 *   3. relays each decoded frame to the car over the nRF24 radio (Stage 3),
 *   4. applies a link-loss failsafe: if no valid frame arrives for
 *      APP_FAILSAFE_MS, the car is treated as disarmed (stopped).
 *
 * All pins/ports/RF parameters live in app_config.h.
 *
 * LD2 shows link/arm status at a glance:
 *   - off        : link lost (failsafe) or startup
 *   - slow blink : link OK but disarmed
 *   - solid on   : link OK and armed
 ******************************************************************************
 */
#include "rcc.h"
#include "gpio.h"
#include "uart.h"
#include "spi.h"
#include "systick.h"
#include "rf_protocol.h"
#include "nrf24.h"
#include "app_config.h"

#include <string.h>

#define forever for (;;)

/* Latest received command + bookkeeping for the failsafe. */
static rf_control_frame_t s_latest;
static uint32_t s_last_rx_ms;
static bool s_have_frame;
static bool s_radio_ok;
static bool s_clock_ok;   /* did the PLL actually come up at the target speed? */

/* ---- nRF24 hardware hooks (wrap the station drivers for the shared driver) */
static void nrf_spi(const uint8_t *tx, uint8_t *rx, size_t len)
{
    (void)spi_transfer(APP_NRF_SPI, tx, rx, len);
}
static void nrf_csn(bool high)
{
    gpio_write(APP_NRF_CSN_PORT, APP_NRF_CSN_PIN, high ? GPIO_HIGH : GPIO_LOW);
}
static void nrf_ce(bool high)
{
    gpio_write(APP_NRF_CE_PORT, APP_NRF_CE_PIN, high ? GPIO_HIGH : GPIO_LOW);
}
static void nrf_delay(uint32_t us)
{
    systick_delay_us(us);
}
static const nrf24_hal_t s_nrf_hal = {
    .spi_transfer = nrf_spi,
    .csn_write    = nrf_csn,
    .ce_write     = nrf_ce,
    .delay_us     = nrf_delay,
};
static nrf24_t s_radio;

/* Put one pin into alternate-function mode. The SPI2 signals live on different
 * ports and may use different AF indices, so each is configured on its own. */
static void af_pin_init(GPIO_TypeDef *port, uint8_t pin, gpio_af_t af)
{
    const gpio_config_t cfg = {
        .pin_mask = GPIO_PIN(pin),
        .mode  = GPIO_MODE_AF,
        .otype = GPIO_OTYPE_PP,
        .speed = GPIO_SPEED_VERY_HIGH,
        .pull  = GPIO_PULL_NONE,
        .af    = af,
    };
    gpio_init(port, &cfg);
}

static void radio_init(void)
{
    /* SPI SCK/MOSI/MISO -> alternate function (per-pin port + AF). */
    af_pin_init(APP_NRF_SCK_PORT,  APP_NRF_SCK_PIN,  APP_NRF_SCK_AF);
    af_pin_init(APP_NRF_MOSI_PORT, APP_NRF_MOSI_PIN, APP_NRF_MOSI_AF);
    af_pin_init(APP_NRF_MISO_PORT, APP_NRF_MISO_PIN, APP_NRF_MISO_AF);

    const spi_config_t spi_cfg = {
        .mode      = SPI_MODE0,
        .baud      = APP_NRF_SPI_BAUD,
        .bit_order = SPI_BIT_MSB_FIRST,
        .datasize  = SPI_DATASIZE_8BIT,
    };
    spi_init(APP_NRF_SPI, &spi_cfg);

    /* Control lines: CSN idles high (deselected), CE idles low. */
    gpio_init_output(APP_NRF_CSN_PORT, APP_NRF_CSN_PIN, GPIO_OTYPE_PP, GPIO_HIGH);
    gpio_init_output(APP_NRF_CE_PORT, APP_NRF_CE_PIN, GPIO_OTYPE_PP, GPIO_LOW);
    gpio_init_input(APP_NRF_IRQ_PORT, APP_NRF_IRQ_PIN, GPIO_PULL_UP);

    static const uint8_t addr[] = APP_RF_ADDRESS;
    nrf24_config_t rf_cfg = {
        .channel       = APP_RF_CHANNEL,
        .data_rate     = APP_RF_DATARATE,
        .power         = APP_RF_POWER,
        .payload_width = APP_RF_PAYLOAD,
        .auto_ack      = APP_RF_AUTO_ACK,
    };
    memcpy(rf_cfg.address, addr, NRF24_ADDR_WIDTH);

    s_radio_ok = nrf24_init(&s_radio, &s_nrf_hal, &rf_cfg);
    if (s_radio_ok) {
        nrf24_set_tx_mode(&s_radio);
    }
}

static void board_init(void)
{
    /* 180 MHz. APP_HSE_HZ = 0 uses the internal HSI (dependency-free).*/
    s_clock_ok = (rcc_sysclk_config_default(APP_HSE_HZ) == DRV_OK);
    systick_init(0); /* 1 ms tick, derived from the real HCLK */

    gpio_init_output(APP_LED_PORT, APP_LED_PIN, GPIO_OTYPE_PP, GPIO_LOW);

    /* Command UART pins -> alternate function. */
    const gpio_config_t uart_pins = {
        .pin_mask = GPIO_PIN(APP_UART_TX_PIN) | GPIO_PIN(APP_UART_RX_PIN),
        .mode  = GPIO_MODE_AF,
        .otype = GPIO_OTYPE_PP,
        .speed = GPIO_SPEED_VERY_HIGH,
        .pull  = GPIO_PULL_UP,
        .af    = APP_UART_AF,
    };
    gpio_init(APP_UART_GPIO, &uart_pins);
    uart_init_8n1(APP_UART, APP_UART_BAUD);

    radio_init();
}

/* Returns true while frames are arriving on time. */
static bool link_ok(void)
{
    return s_have_frame && !systick_elapsed(s_last_rx_ms, APP_FAILSAFE_MS);
}

/**
 * @brief Fault-code blink: @p count short flashes, then a long gap, repeating.
 *
 * Same idea as BIOS beep codes — countable at a glance, and clearly distinct
 * from the steady patterns used for normal operation.
 */
static bool blink_code(uint32_t ms, uint32_t count)
{
    const uint32_t on = 150u, off = 150u, gap = 900u;
    const uint32_t burst = count * (on + off);
    const uint32_t t = ms % (burst + gap);
    return (t < burst) && ((t % (on + off)) < on);
}

/**
 * @brief Drive LD2 so that every failure mode is distinguishable.
 *
 *   off                : firmware not running (crash, hang, no power)
 *   2 flashes + pause  : clock/PLL configuration failed
 *   3 flashes + pause  : nRF24 init failed (check wiring/power)
 *   short heartbeat    : running, waiting for frames from the laptop
 *   fast blink (~4 Hz) : link up, car disarmed
 *   solid              : link up, car armed
 */
static void update_status_led(void)
{
    const uint32_t ms = systick_get_ms();
    bool on;

    if (!s_clock_ok) {
        on = blink_code(ms, 2u);
    } else if (!s_radio_ok) {
        on = blink_code(ms, 3u);
    } else if (!link_ok()) {
        on = (ms % 1000u) < 60u;              /* brief flash once a second */
    } else if (rf_control_is_armed(&s_latest)) {
        on = true;
    } else {
        on = (ms & 0x80u) != 0u;              /* ~4 Hz */
    }

    gpio_write(APP_LED_PORT, APP_LED_PIN, on ? GPIO_HIGH : GPIO_LOW);
}

int main(void)
{
    board_init();

    rf_control_parser_t parser;
    rf_control_parser_reset(&parser);

    forever {
        /* Drain everything the UART has received this pass. */
        while (uart_rx_ready(APP_UART)) {
            uint8_t byte = uart_read_byte(APP_UART);

            rf_control_frame_t frame;
            if (rf_control_parser_feed(&parser, byte, &frame)) {
                s_latest = frame;
                s_last_rx_ms = systick_get_ms();
                s_have_frame = true;

                /* Stage 3: relay the frame to the car over the radio. The
                 * struct is packed to the exact 7-byte wire layout. */
                if (s_radio_ok) {
                    (void)nrf24_send(&s_radio, (const uint8_t *)&s_latest,
                                     APP_RF_PAYLOAD);
                }
            }
        }

        update_status_led();
    }
}
