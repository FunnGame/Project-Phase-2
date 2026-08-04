/**
 ******************************************************************************
 * @file    main.c
 * @brief   Car RF gateway node: receives control frames over nRF24.
 *
 * Runs on the STM32F103. The station (Nucleo-F446RE) relays 7-byte control
 * frames (contracts/rf_protocol.h) from the laptop over the radio; this node:
 *
 *   1. brings up the clock, tick, LEDs, SPI2 and the nRF24 in receive mode,
 *   2. reads each payload and validates magic + CRC,
 *   3. tracks the sequence counter to count dropped and duplicate frames,
 *   4. applies the command to the outputs, and
 *   5. applies two independent safety rules:
 *        - link-loss failsafe: no valid frame for CAR_FAILSAFE_MS => stop
 *        - disarmed (armed bit clear) => stop
 *
 * All pins and RF parameters live in car_config.h.
 *
 * Indicator LEDs (the TB6612 motors are driven by car/devices/tb6612, which
 * this node does not yet call — see the TODO in outputs_apply):
 *   - status LED  : PA5  — off = failsafe, blink = disarmed, solid = armed
 *   - drive LED   : PB0  — PWM brightness follows THROTTLE while armed
 *   - reverse LED : PB5  — lit while the REVERSE bit is set
 *
 * Build: compile with car/drivers, shared/nrf24/nrf24.c, contracts/
 * rf_protocol.c, platform/f103/system_stm32f1xx.c and the F103 startup +
 * linker script. Define STM32F103xB.
 ******************************************************************************
 */
#include "drv_common.h"
#include "drv_clock.h"
#include "drv_gpio.h"
#include "drv_spi.h"
#include "drv_pwm.h"
#include "drv_timer.h"
#include "drv_systick.h"
#include "nrf24.h"
#include "rf_protocol.h"
#include "car_config.h"
#include "control_app.h"    /* differential-drive mixer (car/lib/mixer)         */

#include <string.h>

#define forever for (;;)

/* SWD diagnostic: a RAM snapshot of the clocks and nRF24 registers, read over
 * SWD by `cmake --build build --target diag-car` (no UART on this board). Set
 * to 0 (or build -DCAR_DIAG=0) to strip it once bring-up is done. */
#ifndef CAR_DIAG
#define CAR_DIAG 1
#endif

/* ---- Received-command state --------------------------------------------- */
static rf_control_frame_t s_latest;      /* last valid frame                 */
static uint32_t s_last_rx_ms;            /* when it arrived                  */
static bool     s_have_frame;            /* anything received yet?           */
static bool     s_radio_ok;              /* did the radio initialise?        */
static bool     s_clock_ok;              /* did the PLL reach the target?    */

/* ---- Sequence tracking ---------------------------------------------------
 * The counters are not read by the firmware today; they are kept because they
 * are the link-quality figures the planned telemetry display will show, and
 * because they cost two words of RAM. */
static uint8_t  s_last_seq;
static bool     s_seq_valid;
static uint32_t s_dropped;               /* frames lost in transit           */
static uint32_t s_duplicates;            /* retransmits we already applied   */

/* -------------------------------------------------------------------------- */
/*  nRF24 hardware hooks (wrap the car drivers for the shared radio driver)   */
/* -------------------------------------------------------------------------- */

static void nrf_spi(const uint8_t *tx, uint8_t *rx, size_t len)
{
    (void)DRV_SPI_TransferBuffer(CAR_NRF_SPI, tx, rx, len);
}
static void nrf_csn(bool high)
{
    DRV_GPIO_Write(CAR_NRF_CSN_PORT, CAR_NRF_CSN_PIN,
                   high ? GPIO_HIGH : GPIO_LOW);
}
static void nrf_ce(bool high)
{
    DRV_GPIO_Write(CAR_NRF_CE_PORT, CAR_NRF_CE_PIN,
                   high ? GPIO_HIGH : GPIO_LOW);
}
static void nrf_delay(uint32_t us)
{
    DRV_Delay_Us(us);
}

static const nrf24_hal_t s_nrf_hal = {
    .spi_transfer = nrf_spi,
    .csn_write    = nrf_csn,
    .ce_write     = nrf_ce,
    .delay_us     = nrf_delay,
};
static nrf24_t s_radio;

/* -------------------------------------------------------------------------- */
/*  Output helpers                                                            */
/* -------------------------------------------------------------------------- */

/* The Blue Pill's LED sinks current, so the logical sense may be inverted. */
static void led_status_write(bool on)
{
#if CAR_LED_ACTIVE_LOW
    DRV_GPIO_Write(CAR_LED_STATUS_PORT, CAR_LED_STATUS_PIN,
                   on ? GPIO_LOW : GPIO_HIGH);
#else
    DRV_GPIO_Write(CAR_LED_STATUS_PORT, CAR_LED_STATUS_PIN,
                   on ? GPIO_HIGH : GPIO_LOW);
#endif
}

/* True while frames are arriving on time. */
static bool link_ok(void)
{
    return s_have_frame && !DRV_SysTick_Elapsed(s_last_rx_ms, CAR_FAILSAFE_MS);
}

/* Everything off. Called on link loss and whenever the car is disarmed. */
static void outputs_stop(void)
{
    ControlApp_Stop();                     /* coast both wheels */
    DRV_PWM_SetDuty(CAR_DRIVE_TIMER, CAR_DRIVE_CHANNEL, 0u);   /* drive LED off */
    DRV_GPIO_Clear(CAR_REVERSE_LED_PORT, CAR_REVERSE_LED_PIN);
}

/**
 * @brief Drive the outputs from the latest command.
 *
 * Converts the frame into a single motion intent and hands it to the mixer,
 * which splits it across the two wheels. The frame->intent translation and the
 * safety gates live here (node-specific); the kinematics live in the mixer.
 */
static void outputs_apply(const rf_control_frame_t *frame)
{
    /* Safety gate: a disarmed car ignores drive commands entirely. */
    if (!rf_control_is_armed(frame)) {
        outputs_stop();
        return;
    }

    /* The frame carries throttle as an unsigned magnitude plus a reverse bit;
     * the mixer wants a signed throttle. Brake wins over throttle — pressing
     * both pedals stops rather than drives. */
    const bool reverse = rf_control_is_reverse(frame);
    int8_t throttle = (frame->brake > 0u) ? 0
                    : reverse ? -(int8_t)frame->throttle
                    :            (int8_t)frame->throttle;

    ControlApp_Drive(throttle, frame->steering);

    /* Indicators: drive LED brightness tracks throttle magnitude, reverse LED
     * mirrors the direction bit. */
    DRV_PWM_SetDuty(CAR_DRIVE_TIMER, CAR_DRIVE_CHANNEL,
                    (frame->brake > 0u) ? 0u : frame->throttle);
    DRV_GPIO_Write(CAR_REVERSE_LED_PORT, CAR_REVERSE_LED_PIN,
                   reverse ? GPIO_HIGH : GPIO_LOW);

    /* Brake is treated as "cut throttle" (coast) for now. Active braking via
     * TB6612_Brake() is available but deliberately deferred to the AEB stage,
     * where the warning/partial/full cascade decides when to short the motors. */
}

/**
 * @brief Fault-code blink: @p count short flashes, then a long gap, repeating.
 *        Countable at a glance, and clearly distinct from the steady patterns.
 */
static bool blink_code(uint32_t ms, uint32_t count)
{
    const uint32_t on = 150u, off = 150u, gap = 900u;
    const uint32_t burst = count * (on + off);
    const uint32_t t = ms % (burst + gap);
    return (t < burst) && ((t % (on + off)) < on);
}

/**
 * @brief Drive the status LED so every failure mode is distinguishable.
 *
 *   off                : firmware not running (crash, hang, no power)
 *   2 flashes + pause  : clock/PLL configuration failed — suspect the HSE
 *                        crystal if CAR_HSE_HZ is non-zero
 *   3 flashes + pause  : nRF24 init failed (check wiring and the 3V3 supply)
 *   short heartbeat    : running, waiting for frames from the station
 *   fast blink (~4 Hz) : link up, disarmed
 *   solid              : link up, armed
 */
static void status_led_update(void)
{
    const uint32_t ms = DRV_SysTick_GetTick();
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

    led_status_write(on);
}

/* -------------------------------------------------------------------------- */
/*  Initialisation                                                            */
/* -------------------------------------------------------------------------- */

static void radio_init(void)
{
    const SPI_Config spi_cfg = {
        .spi       = CAR_NRF_SPI,
        .port      = CAR_NRF_SPI_PORT,
        .sck_pin   = CAR_NRF_SCK_PIN,
        .miso_pin  = CAR_NRF_MISO_PIN,
        .mosi_pin  = CAR_NRF_MOSI_PIN,
        .mode      = SPI_MODE0,
        .baud      = CAR_NRF_SPI_BAUD,
        .bit_order = SPI_MSB_FIRST,
    };
    DRV_SPI_Init(&spi_cfg);

    /* CSN idles high (deselected), CE idles low (not listening yet). */
    DRV_SPI_InitCS(CAR_NRF_CSN_PORT, CAR_NRF_CSN_PIN);
    DRV_GPIO_InitOutput(CAR_NRF_CE_PORT, CAR_NRF_CE_PIN, GPIO_LOW);
    DRV_GPIO_InitInput(CAR_NRF_IRQ_PORT, CAR_NRF_IRQ_PIN, GPIO_PULL_UP);

    static const uint8_t addr[] = CAR_RF_ADDRESS;
    nrf24_config_t rf_cfg = {
        .channel       = CAR_RF_CHANNEL,
        .data_rate     = CAR_RF_DATARATE,
        .power         = CAR_RF_POWER,
        .payload_width = CAR_RF_PAYLOAD,
        .auto_ack      = CAR_RF_AUTO_ACK,
    };
    memcpy(rf_cfg.address, addr, NRF24_ADDR_WIDTH);

    s_radio_ok = nrf24_init(&s_radio, &s_nrf_hal, &rf_cfg);
    if (s_radio_ok) {
        nrf24_set_rx_mode(&s_radio);   /* start listening */
    }
}

#if CAR_DIAG
/* -------------------------------------------------------------------------- */
/*  SWD diagnostic snapshot                                                    */
/* -------------------------------------------------------------------------- */

/* Every field is a uint32_t so the host side reads exactly one word per field
 * from an `mdw` dump — no packing or endianness to unravel. `used` + volatile
 * stop -Os discarding a struct the firmware itself never reads back. */
#define CAR_DIAG_MAGIC  0xD1A6C0DEu

typedef struct {
    uint32_t magic;         /* CAR_DIAG_MAGIC — confirms the read location   */
    uint32_t sysclk_hz;     /* expect 72000000                               */
    uint32_t pclk1_hz;      /* expect 36000000 (this feeds SPI2)             */
    uint32_t timer_hz;      /* TIM3 kernel clock                             */
    uint32_t clock_ok;      /* expect 1                                      */
    uint32_t radio_ok;      /* expect 1                                      */
    uint32_t nrf_config;    /* expect 0x0F (RX: EN_CRC|CRCO|PWR_UP|PRIM_RX)  */
    uint32_t nrf_en_aa;     /* expect 0x01                                   */
    uint32_t nrf_setup_retr;/* expect 0x1F                                   */
    uint32_t nrf_rf_ch;     /* expect 0x4C (76)                              */
    uint32_t nrf_rf_setup;  /* expect 0x06 (1 Mbps, 0 dBm)                   */
    uint32_t nrf_status;    /* expect 0x0E on an idle module                 */
    uint32_t nrf_rx_pw_p0;  /* expect 0x07 (payload width)                   */
} car_diag_t;

static volatile car_diag_t s_diag __attribute__((used));

/* Snapshot the clocks and read back the seven nRF24 registers. Reads happen
 * even when the presence check failed — that is the whole point: all-0x00 =>
 * MISO unwired / module unpowered, all-0xFF => MISO floating. */
static void diag_capture(void)
{
    s_diag.magic     = CAR_DIAG_MAGIC;
    s_diag.sysclk_hz = DRV_Clock_GetSysClk();
    s_diag.pclk1_hz  = DRV_Clock_GetPCLK1();
    s_diag.timer_hz  = DRV_Clock_GetTimerClock(CAR_MOTOR_PWM_TIMER);
    s_diag.clock_ok  = s_clock_ok ? 1u : 0u;
    s_diag.radio_ok  = s_radio_ok ? 1u : 0u;

    s_diag.nrf_config     = nrf24_read_register(&s_radio, NRF24_REG_CONFIG);
    s_diag.nrf_en_aa      = nrf24_read_register(&s_radio, NRF24_REG_EN_AA);
    s_diag.nrf_setup_retr = nrf24_read_register(&s_radio, NRF24_REG_SETUP_RETR);
    s_diag.nrf_rf_ch      = nrf24_read_register(&s_radio, NRF24_REG_RF_CH);
    s_diag.nrf_rf_setup   = nrf24_read_register(&s_radio, NRF24_REG_RF_SETUP);
    s_diag.nrf_status     = nrf24_read_register(&s_radio, NRF24_REG_STATUS);
    s_diag.nrf_rx_pw_p0   = nrf24_read_register(&s_radio, NRF24_REG_RX_PW_P0);
}
#endif /* CAR_DIAG */

static void board_init(void)
{
    /* Keep the result: a missing or dead HSE crystal makes this time out, and
     * the part carries on at 8 MHz HSI. Everything still runs, just 9x slow —
     * an easy failure to miss, so the LED reports it as a 2-flash code. */
    s_clock_ok = (DRV_Clock_Init72MHz(CAR_HSE_HZ) == DRV_OK);
    DRV_Delay_Init();                                  /* DWT, for the radio */

    /* The 1 ms tick runs on SysTick, not a general-purpose timer: all four of
     * those are spoken for (see the timer budget in car_config.h). */
    DRV_SysTick_Init(CAR_TICK_IRQ_PRIORITY);

    /* Status LED: drive the inactive level first so it never flashes on reset,
     * then switch the pin to an output.
     *
     * 2 MHz rather than DRV_GPIO_InitOutput()'s 50 MHz: an LED needs no slew
     * rate, slower edges radiate less, and it keeps the code within spec if the
     * LED is ever moved back to PC13 (DS5319 Table 5 note 5 caps PC13-PC15 at
     * 2 MHz / 30 pF). */
    DRV_GPIO_Write(CAR_LED_STATUS_PORT, CAR_LED_STATUS_PIN,
                   CAR_LED_ACTIVE_LOW ? GPIO_HIGH : GPIO_LOW);
    DRV_GPIO_Init(CAR_LED_STATUS_PORT, CAR_LED_STATUS_PIN,
                  GPIO_MODE_OUT_2M, GPIO_CNF_OUT_PP, GPIO_PULL_NONE);

    DRV_GPIO_InitOutput(CAR_REVERSE_LED_PORT, CAR_REVERSE_LED_PIN, GPIO_LOW);

    /* Drive output: PWM, starts at 0 %. */
    const PWM_Config drive = {
        .tim       = CAR_DRIVE_TIMER,
        .channel   = CAR_DRIVE_CHANNEL,
        .port      = CAR_DRIVE_PORT,
        .pin       = CAR_DRIVE_PIN,
        .frequency = CAR_MOTOR_PWM_HZ,
        .polarity  = PWM_ACTIVE_HIGH,
    };
    DRV_PWM_Init(&drive);
    DRV_PWM_Start(CAR_DRIVE_TIMER, CAR_DRIVE_CHANNEL);

    /* Motors: configures the TB6612 direction pins, STBY (driven high) and both
     * PWM channels, and leaves the car stopped. From here the firmware owns
     * PA4/STBY — do not also drive it externally. Shares TIM3 with the drive
     * LED above at the same 20 kHz, so the order of the two is immaterial. */
    ControlApp_Init();

    radio_init();

#if CAR_DIAG
    diag_capture();      /* snapshot for SWD readout (target diag-car) */
#endif
}

/* -------------------------------------------------------------------------- */
/*  Frame handling                                                            */
/* -------------------------------------------------------------------------- */

/* Update the drop/duplicate statistics from this frame's sequence number.
 * Returns false if the frame is a duplicate and should not be re-applied. */
static bool sequence_track(uint8_t seq)
{
    if (!s_seq_valid) {
        s_last_seq  = seq;
        s_seq_valid = true;
        return true;
    }

    /* Unsigned subtraction is wrap-safe across the 255 -> 0 rollover. */
    const uint8_t gap = (uint8_t)(seq - s_last_seq);
    if (gap == 0u) {
        s_duplicates++;          /* auto-ACK retransmit we already handled */
        return false;
    }
    s_dropped += (uint32_t)(gap - 1u);
    s_last_seq = seq;
    return true;
}

static void radio_poll(void)
{
    uint8_t buf[RF_CONTROL_FRAME_SIZE];

    while (nrf24_data_available(&s_radio)) {
        if (!nrf24_read(&s_radio, buf, CAR_RF_PAYLOAD)) {
            break;
        }

        rf_control_frame_t frame;
        memcpy(&frame, buf, sizeof(frame));

        /* Reject anything corrupted on the air. */
        if (!rf_control_frame_valid(&frame)) {
            continue;
        }

        s_latest     = frame;
        s_last_rx_ms = DRV_SysTick_GetTick();
        s_have_frame = true;

        if (sequence_track(frame.seq)) {
            outputs_apply(&frame);
        }
    }
}

int main(void)
{
    board_init();

    forever {
        if (s_radio_ok) {
            radio_poll();
        }

        /* Link lost: stop regardless of the last command received. */
        if (!link_ok()) {
            outputs_stop();
        }

        status_led_update();
    }
}
