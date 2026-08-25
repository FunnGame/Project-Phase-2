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
#include "ili9341.h"
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
        .ack_payload   = APP_RF_ACK_PAYLOAD,
    };
    memcpy(rf_cfg.address, addr, NRF24_ADDR_WIDTH);

    s_radio_ok = nrf24_init(&s_radio, &s_nrf_hal, &rf_cfg);
    if (s_radio_ok) {
        nrf24_set_tx_mode(&s_radio);
    }
}

static void tft_init(void);   /* defined with the rest of the display code */

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
    tft_init();
}

/* ---- Telemetry from the car (rides back on the auto-ACK) ----------------- */
static rf_telemetry_frame_t s_telem;      /* last VALID frame                 */
static uint32_t s_last_telem_ms;          /* when it arrived                  */
static bool     s_have_telem;
/* Two ENTIRELY different faults, so they are counted apart:
 *   s_telem_badlen - the payload was the wrong size. That is a version skew:
 *                    the gateway and this station disagree about
 *                    RF_TELEM_FRAME_SIZE, i.e. one of them was not reflashed.
 *   s_telem_badcrc - right size, failed magic or CRC. That is real corruption
 *                    on the air, and points at RF conditions, not firmware.
 * Lumping them into one number said "something is wrong" and nothing more. */
static uint32_t s_telem_badlen;
static uint32_t s_telem_badcrc;
static uint16_t s_rtt_ms;                 /* round trip, from the seq echo     */

/** @brief True while telemetry is arriving on time. */
static bool telem_ok(void)
{
    return s_have_telem && !systick_elapsed(s_last_telem_ms, APP_TELEM_TIMEOUT_MS);
}

/**
 * @brief Collect whatever came back with the ACK of the frame just sent.
 * @param seq_sent Sequence number of that frame, for the round-trip estimate.
 *
 * A false return from nrf24_read_ack_payload() is NORMAL - the car only has a
 * payload queued once it has received a frame, so the very first exchange after
 * power-up carries nothing.
 */
static void telemetry_poll(uint8_t seq_sent, uint32_t sent_ms)
{
    uint8_t buf[NRF24_MAX_PAYLOAD];
    uint8_t len = 0u;

    if (!nrf24_read_ack_payload(&s_radio, buf, sizeof(buf), &len)) {
        return;
    }
    if (len != RF_TELEM_FRAME_SIZE) {
        s_telem_badlen++;
        return;
    }

    rf_telemetry_frame_t t;
    memcpy(&t, buf, sizeof(t));
    if (!rf_telemetry_frame_valid(&t)) {
        s_telem_badcrc++;
        return;
    }

    s_telem         = t;
    s_last_telem_ms = systick_get_ms();
    s_have_telem    = true;

    /* The car echoes the sequence number it is acknowledging, so the round trip
     * is measurable here with no clock synchronisation at all. Only meaningful
     * when the echo matches the frame we just sent - otherwise this payload was
     * queued in response to an earlier one. */
    if (t.seq_echo == seq_sent) {
        const uint32_t dt = s_last_telem_ms - sent_ms;
        s_rtt_ms = (uint16_t)((dt > 65535u) ? 65535u : dt);
    }
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
 * Ordered worst-first, so the most serious condition always wins the LED.
 * Everything above "link up" is a fault; everything below is normal running.
 *
 *   off                : firmware not running (crash, hang, no power)
 *   2 flashes + pause  : clock/PLL configuration failed
 *   3 flashes + pause  : nRF24 init failed (check wiring/power)
 *   4 flashes + pause  : radio up and frames going out, but NOTHING coming
 *                        back - the car is not acknowledging, or its telemetry
 *                        is corrupt. Distinguishes "car absent" from "car
 *                        present but silent", which a TX-only station cannot.
 *   5 flashes + pause  : telemetry arriving, but the CAR reports a fault
 *                        (VC_Faults non-zero, or a node missing from the bus)
 *   short heartbeat    : running, waiting for frames from the laptop
 *   fast blink (~4 Hz) : link up, car disarmed
 *   solid              : link up, car armed
 *
 * This is the whole diagnostic surface until the ILI9341 lands - it has to
 * carry the car's health as well as the station's, which is why two of the
 * codes describe conditions on the far end of the radio link.
 */
static void update_status_led(void)
{
    const uint32_t ms = systick_get_ms();
    bool on;

    /* The car is faulty if it says so, or if a node it should hear is missing.
     * Only meaningful while telemetry is actually arriving. */
    const bool car_faulted =
        telem_ok() && ((s_telem.faults != 0u) ||
                       rf_telem_degraded(&s_telem) ||
                       ((s_telem.health & RF_TELEM_NODE_VC) == 0u));

    if (!s_clock_ok) {
        on = blink_code(ms, 2u);
    } else if (!s_radio_ok) {
        on = blink_code(ms, 3u);
    } else if (link_ok() && !telem_ok()) {
        on = blink_code(ms, 4u);              /* sending, hearing nothing back */
    } else if (car_faulted) {
        on = blink_code(ms, 5u);
    } else if (!link_ok()) {
        on = (ms % 1000u) < 60u;              /* brief flash once a second */
    } else if (rf_control_is_armed(&s_latest)) {
        on = true;
    } else {
        on = (ms & 0x80u) != 0u;              /* ~4 Hz */
    }

    gpio_write(APP_LED_PORT, APP_LED_PIN, on ? GPIO_HIGH : GPIO_LOW);
}

/* ========================================================================== */
/*  ILI9341 telemetry display                                                 */
/* ========================================================================== */
/*
 * 320x240 landscape dashboard. Nothing here composes a framebuffer - a full
 * RGB565 image is 150 KB against 128 KB of SRAM - so the static chrome is
 * painted once at startup and each cycle repaints only the fields whose value
 * actually changed. A full repaint is ~60 ms of blocking SPI; a changed field
 * is well under a millisecond, which is what keeps the radio loop responsive.
 */

/* Palette. Colour never carries meaning alone: every state that changes colour
 * also changes its text, because the panel's viewing angle in daylight is not
 * something to bet a safety indication on. */
#define TFT_BG          ILI9341_BLACK
#define TFT_LABEL       ILI9341_GREY
#define TFT_VALUE       ILI9341_WHITE
#define TFT_RULE        ILI9341_DARKGREY
#define TFT_OK          ILI9341_GREEN
#define TFT_IDLE        ILI9341_DARKGREY

/* Layout. Grouped so the screen can be rearranged without touching draw code. */
#define TFT_STATE_X     6u
#define TFT_STATE_Y     12u
#define TFT_AEB_X       162u
#define TFT_AEB_Y       18u
/* Drive direction, right of the AEB state. 254 + 3 cells x 12 px = 254..289,
 * inside the 320 px edge. */
#define TFT_DIR_X       254u
#define TFT_RULE1_Y     46u

#define TFT_SPEED_X     8u
#define TFT_RANGE_X     168u
#define TFT_NUM_LBL_Y   52u
#define TFT_NUM_Y       64u
#define TFT_RULE2_Y     104u

#define TFT_THR_X       34u
#define TFT_BRK_X       194u
#define TFT_BAR_W       100u
#define TFT_BAR_H       10u
#define TFT_BAR_CMD_Y   110u
#define TFT_BAR_APP_Y   124u
#define TFT_RULE3_Y     140u

#define TFT_NODE_Y      152u
#define TFT_SEG_X       134u
#define TFT_SEG_W       12u
#define TFT_SEG_PITCH   14u
#define TFT_FLT_X       218u
#define TFT_DIAG_Y      176u
#define TFT_RTT_X       32u
#define TFT_BAD_X       124u
#define TFT_LEN_X       214u

/* IMU cross-check row. Deliberately at the bottom in small type: it is a
 * bring-up instrument, not a driving instrument, and must not compete with the
 * speed and range fields above it. */

static bool s_tft_ok;

/** Last value drawn into each field. The whole dirty-region scheme is this. */
static struct {
    bool     painted;      /**< false until the first update has run        */
    bool     live;         /**< was telemetry fresh last time we drew?      */
    uint8_t  vstate;
    uint8_t  aeb;
    int16_t  speed;
    uint16_t range;
    uint8_t  thr_cmd, brk_cmd, thr_app, brk_app;
    uint8_t  health;
    uint8_t  faults;
    uint16_t rtt;
    uint16_t bad;
    bool     vc_heard;
    uint16_t badlen;
    uint8_t  dir;          /* 0 fwd, 1 rev, 2 unknown */
} s_shadow;

static uint32_t s_tft_last_ms;

/* ---- formatting ---------------------------------------------------------- */
/* Hand-rolled rather than snprintf: stdio would pull several KB of newlib into
 * a firmware whose entire formatting need is a handful of short numbers. */

static void fmt_u16(char *buf, uint16_t v)          /* buf >= 6 bytes */
{
    char    tmp[5];
    uint8_t n = 0u;

    do {
        tmp[n++] = (char)('0' + (v % 10u));
        v = (uint16_t)(v / 10u);
    } while (v != 0u);

    for (uint8_t i = 0u; i < n; ++i) {
        buf[i] = tmp[n - 1u - i];
    }
    buf[n] = '\0';
}

static void fmt_i16(char *buf, int16_t v)           /* buf >= 7 bytes */
{
    if (v < 0) {
        buf[0] = '-';
        fmt_u16(&buf[1], (uint16_t)(-(int32_t)v));
    } else {
        fmt_u16(buf, (uint16_t)v);
    }
}

static void fmt_hex8(char *buf, uint8_t v)          /* buf >= 3 bytes */
{
    static const char hex[] = "0123456789ABCDEF";

    buf[0] = hex[(v >> 4) & 0x0Fu];
    buf[1] = hex[v & 0x0Fu];
    buf[2] = '\0';
}

/* ---- state names and colours -------------------------------------------- */

static const char *vehicle_state_name(uint8_t s)
{
    static const char *const names[] = {
        "INIT", "FAILSAFE", "DISARMED", "ARMED", "DEGRADED", "FAULT",
    };
    return (s < DRV_ARRAY_LEN(names)) ? names[s] : "?";
}

static uint16_t vehicle_state_color(uint8_t s)
{
    switch (s) {
    case 1u: return ILI9341_RED;      /* FAILSAFE */
    case 3u: return ILI9341_RED;      /* ARMED - the car can move           */
    case 4u: return ILI9341_ORANGE;   /* DEGRADED */
    case 5u: return ILI9341_RED;      /* FAULT */
    default: return TFT_LABEL;        /* INIT, DISARMED                     */
    }
}

static const char *aeb_state_name(uint8_t s)
{
    static const char *const names[] = {
        "IDLE", "WARN", "PARTIAL", "FULL", "LATCHED",
    };
    return (s < DRV_ARRAY_LEN(names)) ? names[s] : "?";
}

static uint16_t aeb_state_color(uint8_t s)
{
    switch (s) {
    case 1u: return ILI9341_YELLOW;   /* WARN    */
    case 2u: return ILI9341_ORANGE;   /* PARTIAL */
    case 3u:
    case 4u: return ILI9341_RED;      /* FULL, LATCHED */
    default: return TFT_IDLE;         /* IDLE */
    }
}

/* ---- drawing helpers ----------------------------------------------------- */

/** Two-tone horizontal bar: @p pct of @p w filled, the remainder cleared. */
static void tft_bar(uint16_t x, uint16_t y, uint8_t pct, uint16_t fill)
{
    if (pct > 100u) {
        pct = 100u;
    }
    const uint16_t on = (uint16_t)(((uint32_t)TFT_BAR_W * pct) / 100u);

    if (on != 0u) {
        ili9341_fill_rect(x, y, on, TFT_BAR_H, fill);
    }
    if (on < TFT_BAR_W) {
        ili9341_fill_rect((uint16_t)(x + on), y,
                          (uint16_t)(TFT_BAR_W - on), TFT_BAR_H, TFT_IDLE);
    }
}

/** Static chrome: labels and rules that never change after startup. */
static void tft_paint_chrome(void)
{
    ili9341_fill_screen(TFT_BG);

    ili9341_draw_text(TFT_AEB_X, 6u, "AEB", 1u, TFT_LABEL, TFT_BG);
    ili9341_draw_text(TFT_DIR_X, 6u, "DIR", 1u, TFT_LABEL, TFT_BG);
    ili9341_draw_hline(0u, TFT_RULE1_Y, ili9341_width(), TFT_RULE);

    ili9341_draw_text(TFT_SPEED_X, TFT_NUM_LBL_Y, "SPEED mm/s", 1u,
                      TFT_LABEL, TFT_BG);
    ili9341_draw_text(TFT_RANGE_X, TFT_NUM_LBL_Y, "RANGE mm", 1u,
                      TFT_LABEL, TFT_BG);
    ili9341_draw_hline(0u, TFT_RULE2_Y, ili9341_width(), TFT_RULE);

    /* Two stacked bars per axis: commanded on top, applied below. The gap
     * between them IS the AEB intervention - nothing else on screen shows it. */
    ili9341_draw_text(8u,   TFT_BAR_CMD_Y, "THR", 1u, TFT_LABEL, TFT_BG);
    ili9341_draw_text(8u,   TFT_BAR_APP_Y, "app", 1u, TFT_LABEL, TFT_BG);
    ili9341_draw_text(168u, TFT_BAR_CMD_Y, "BRK", 1u, TFT_LABEL, TFT_BG);
    ili9341_draw_text(168u, TFT_BAR_APP_Y, "app", 1u, TFT_LABEL, TFT_BG);
    ili9341_draw_hline(0u, TFT_RULE3_Y, ili9341_width(), TFT_RULE);

    ili9341_draw_text(110u, (uint16_t)(TFT_NODE_Y + 4u), "LNK", 1u,
                      TFT_LABEL, TFT_BG);
    ili9341_draw_text(196u, (uint16_t)(TFT_NODE_Y + 4u), "FLT", 1u,
                      TFT_LABEL, TFT_BG);

    ili9341_draw_text(8u,   (uint16_t)(TFT_DIAG_Y + 4u), "RTT", 1u,
                      TFT_LABEL, TFT_BG);
    ili9341_draw_text(100u, (uint16_t)(TFT_DIAG_Y + 4u), "CRC", 1u,
                      TFT_LABEL, TFT_BG);
    ili9341_draw_text(190u, (uint16_t)(TFT_DIAG_Y + 4u), "LEN", 1u,
                      TFT_LABEL, TFT_BG);

    /* IMU cross-check row: integrated accelerometer velocity against the
     * encoder speed shown above. VY is the drift meter - a differential-drive
     * chassis cannot move sideways, so whatever VY accumulates on a straight
     * run is integration error, and VX is wrong by about as much. */
}

static void tft_init(void)
{
    af_pin_init(APP_TFT_SCK_PORT,  APP_TFT_SCK_PIN,  APP_TFT_SCK_AF);
    af_pin_init(APP_TFT_MOSI_PORT, APP_TFT_MOSI_PIN, APP_TFT_MOSI_AF);
    /* MISO deliberately unconfigured - the panel is driven write-only. */

    const spi_config_t spi_cfg = {
        .mode      = SPI_MODE0,
        .baud      = APP_TFT_SPI_BAUD,
        .bit_order = SPI_BIT_MSB_FIRST,
        .datasize  = SPI_DATASIZE_8BIT,
    };
    if (spi_init(APP_TFT_SPI, &spi_cfg) != DRV_OK) {
        return;
    }

    const ili9341_hw_t tft = {
        .spi      = APP_TFT_SPI,
        .cs_port  = APP_TFT_CS_PORT,  .cs_pin  = APP_TFT_CS_PIN,
        .dc_port  = APP_TFT_DC_PORT,  .dc_pin  = APP_TFT_DC_PIN,
        .rst_port = APP_TFT_RST_PORT, .rst_pin = APP_TFT_RST_PIN,
        .rotation = APP_TFT_ROTATION,
    };

    /* With SDO unwired there is nothing to read back, so DRV_OK only means the
     * init sequence was sent. The chrome below is the real proof of life. */
    s_tft_ok = (ili9341_init(&tft) == DRV_OK);
    if (s_tft_ok) {
        tft_paint_chrome();
    }
}

/**
 * @brief Repaint whichever telemetry fields changed since the last call.
 *
 * Rate-limited, and each field is compared before it is drawn, so a steady
 * car costs nothing at all. Never called from an interrupt - all display
 * traffic happens at one point in the main loop, where no radio transfer is
 * in flight.
 */
static void tft_update(void)
{
    if (!s_tft_ok || !systick_elapsed(s_tft_last_ms, 100u)) {
        return;
    }
    s_tft_last_ms = systick_get_ms();

    const bool live  = telem_ok();
    const bool first = !s_shadow.painted;
    const bool relive = first || (live != s_shadow.live);

    char buf[8];

    /* ---- safety band ---------------------------------------------------- */
    /* Stale telemetry must never leave a previous ARMED on screen: a corrupted
     * or missing state byte showing DISARMED for an armed car is the one
     * display error that gets somebody hurt (see rf_protocol.h). */
    const uint8_t vstate = live ? rf_telem_vehicle_state(&s_telem) : 0u;
    const uint8_t aeb    = live ? rf_telem_aeb_state(&s_telem)     : 0u;

    /* The gateway sends state 0 to mean "the vehicle node is not talking to
     * me", not "the vehicle node reports INIT" - publish_status() on that node
     * can only ever emit FAILSAFE, ARMED or DISARMED, so a genuine INIT never
     * reaches the air. Rendering 0 through the state-name table printed INIT
     * and made a dead CAN link look like a healthy boot. Trust the node
     * presence bit instead. */
    const bool vc_heard = live && ((s_telem.health & RF_TELEM_NODE_VC) != 0u);

    if (relive || vstate != s_shadow.vstate || vc_heard != s_shadow.vc_heard) {
        if (live && !vc_heard) {
            ili9341_draw_field(TFT_STATE_X, TFT_STATE_Y, 8u, 3u,
                               "NO CAR", false, ILI9341_RED, TFT_BG);
        } else if (live) {
            ili9341_draw_field(TFT_STATE_X, TFT_STATE_Y, 8u, 3u,
                               vehicle_state_name(vstate), false,
                               vehicle_state_color(vstate), TFT_BG);
        } else {
            ili9341_draw_field(TFT_STATE_X, TFT_STATE_Y, 8u, 3u,
                               "NO LINK", false, ILI9341_RED, TFT_BG);
        }
    }

    if (relive || aeb != s_shadow.aeb) {
        ili9341_draw_field(TFT_AEB_X, TFT_AEB_Y, 7u, 2u,
                           live ? aeb_state_name(aeb) : "---", false,
                           live ? aeb_state_color(aeb) : TFT_IDLE, TFT_BG);
    }

    /* ---- drive direction ------------------------------------------------
     * Taken from the frame this station last TRANSMITTED, not from telemetry:
     * it is the operator's own command, and it should appear immediately
     * rather than after a radio round trip.
     *
     * It earns a place on screen because reverse is a LATCHED toggle. While it
     * was a held button your thumb told you the state; now nothing does, and
     * discovering the direction by watching which way the car sets off is not
     * an acceptable way to find out. */
    const uint8_t dir = !link_ok() ? 2u
                      : (rf_control_is_reverse(&s_latest) ? 1u : 0u);

    if (first || dir != s_shadow.dir) {
        static const char *const dir_name[3] = { "FWD", "REV", "---" };
        const uint16_t dir_col = (dir == 1u) ? ILI9341_ORANGE
                               : (dir == 0u) ? TFT_VALUE
                               :               TFT_IDLE;

        ili9341_draw_field(TFT_DIR_X, TFT_AEB_Y, 3u, 2u, dir_name[dir],
                           false, dir_col, TFT_BG);
    }

    /* ---- speed and range ------------------------------------------------ */
    const int16_t  speed = live ? s_telem.speed : 0;
    const uint16_t range = live ? s_telem.range : RF_TELEM_RANGE_NONE;

    if (relive || speed != s_shadow.speed) {
        if (live) { fmt_i16(buf, speed); } else { strcpy(buf, "---"); }
        ili9341_draw_field(TFT_SPEED_X, TFT_NUM_Y, 6u, 4u, buf, true,
                           TFT_VALUE, TFT_BG);
    }

    if (relive || range != s_shadow.range) {
        /* RF_TELEM_RANGE_NONE means "no target, or no sensor node" - a
         * different statement from "far away", so it must not render as
         * 65535. */
        if (live && range != RF_TELEM_RANGE_NONE) {
            fmt_u16(buf, range);
        } else {
            strcpy(buf, "---");
        }
        ili9341_draw_field(TFT_RANGE_X, TFT_NUM_Y, 5u, 4u, buf, true,
                           TFT_VALUE, TFT_BG);
    }

    /* ---- commanded vs applied ------------------------------------------- */
    /* The commanded pair is what this station just transmitted; the applied
     * pair is what the car reports actually reaching the motors. They diverge
     * exactly when the AEB overrides the driver. */
    const uint8_t thr_cmd = link_ok() ? s_latest.throttle : 0u;
    const uint8_t brk_cmd = link_ok() ? s_latest.brake    : 0u;
    const uint8_t thr_app = live ? s_telem.throttle : 0u;
    const uint8_t brk_app = live ? s_telem.brake    : 0u;

    if (relive || thr_cmd != s_shadow.thr_cmd) {
        tft_bar(TFT_THR_X, TFT_BAR_CMD_Y, thr_cmd, TFT_LABEL);
    }
    if (relive || thr_app != s_shadow.thr_app) {
        tft_bar(TFT_THR_X, TFT_BAR_APP_Y, thr_app, TFT_OK);
    }
    if (relive || brk_cmd != s_shadow.brk_cmd) {
        tft_bar(TFT_BRK_X, TFT_BAR_CMD_Y, brk_cmd, TFT_LABEL);
    }
    if (relive || brk_app != s_shadow.brk_app) {
        tft_bar(TFT_BRK_X, TFT_BAR_APP_Y, brk_app, ILI9341_RED);
    }

    /* ---- diagnostics ---------------------------------------------------- */
    const uint8_t health = live ? s_telem.health : 0u;
    const uint8_t faults = live ? s_telem.faults : 0u;

    if (relive || health != s_shadow.health) {
        static const char *const node_name[3] = { "GW", "VC", "SF" };
        static const uint8_t     node_bit[3]  = {
            RF_TELEM_NODE_GW, RF_TELEM_NODE_VC, RF_TELEM_NODE_SF };

        for (uint8_t i = 0u; i < 3u; ++i) {
            const bool present = live && ((health & node_bit[i]) != 0u);
            ili9341_draw_text((uint16_t)(8u + i * 32u), TFT_NODE_Y,
                              node_name[i], 2u,
                              present ? TFT_OK : TFT_IDLE, TFT_BG);
        }

        /* Link quality, four segments off the low nibble (0..15). */
        const uint8_t q = live ? (uint8_t)(health & RF_TELEM_LINKQ_MASK) : 0u;
        for (uint8_t i = 0u; i < 4u; ++i) {
            const bool lit = (q > (uint8_t)(i * 4u));
            ili9341_fill_rect((uint16_t)(TFT_SEG_X + i * TFT_SEG_PITCH),
                              TFT_NODE_Y, TFT_SEG_W, 16u,
                              lit ? TFT_OK : TFT_IDLE);
        }
    }

    if (relive || faults != s_shadow.faults) {
        fmt_hex8(buf, faults);
        ili9341_draw_field(TFT_FLT_X, TFT_NODE_Y, 2u, 2u, buf, false,
                           (faults != 0u) ? ILI9341_RED : TFT_IDLE, TFT_BG);
    }

    /* Round-trip latency - the one metric a CAN trace cannot give you, since
     * it spans the radio link. Feeds the AEB validation protocol directly. */
    if (relive || s_rtt_ms != s_shadow.rtt) {
        if (live) { fmt_u16(buf, s_rtt_ms); } else { strcpy(buf, "---"); }
        ili9341_draw_field(TFT_RTT_X, TFT_DIAG_Y, 4u, 2u, buf, true,
                           TFT_VALUE, TFT_BG);
    }

    const uint16_t bad = (uint16_t)((s_telem_badcrc > 9999u) ? 9999u
                                                             : s_telem_badcrc);
    if (first || bad != s_shadow.bad) {
        fmt_u16(buf, bad);
        ili9341_draw_field(TFT_BAD_X, TFT_DIAG_Y, 4u, 2u, buf, true,
                           (bad != 0u) ? ILI9341_ORANGE : TFT_IDLE, TFT_BG);
    }

    const uint16_t blen = (uint16_t)((s_telem_badlen > 9999u) ? 9999u
                                                              : s_telem_badlen);
    if (first || blen != s_shadow.badlen) {
        fmt_u16(buf, blen);
        /* Red, not amber: a length mismatch is never noise. It means one end
         * is running firmware from before the frame changed size. */
        ili9341_draw_field(TFT_LEN_X, TFT_DIAG_Y, 4u, 2u, buf, true,
                           (blen != 0u) ? ILI9341_RED : TFT_IDLE, TFT_BG);
    }

    s_shadow.painted = true;
    s_shadow.live    = live;
    s_shadow.vstate  = vstate;
    s_shadow.aeb     = aeb;
    s_shadow.speed   = speed;
    s_shadow.range   = range;
    s_shadow.thr_cmd = thr_cmd;
    s_shadow.brk_cmd = brk_cmd;
    s_shadow.thr_app = thr_app;
    s_shadow.brk_app = brk_app;
    s_shadow.health  = health;
    s_shadow.faults  = faults;
    s_shadow.rtt     = s_rtt_ms;
    s_shadow.bad     = bad;
    s_shadow.vc_heard = vc_heard;
    s_shadow.dir     = dir;
    s_shadow.badlen  = blen;
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
                    const uint32_t sent_ms = systick_get_ms();
                    if (nrf24_send(&s_radio, (const uint8_t *)&s_latest,
                                   APP_RF_PAYLOAD) == NRF24_OK) {
                        /* Only an acknowledged frame can carry a payload back,
                         * so this is the one moment worth looking. */
                        telemetry_poll(s_latest.seq, sent_ms);
                    }
                }
            }
        }

        update_status_led();

        /* Display last: one fixed point in the loop, so a pixel burst can
         * never land between a radio send and its ACK read. */
        tft_update();
    }
}
