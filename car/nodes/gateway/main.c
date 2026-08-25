/**
 ******************************************************************************
 * @file    main.c
 * @brief   Gateway node: bridges the 2.4 GHz operator link to the CAN bus.
 *
 * Runs on an STM32F103. The station (Nucleo-F446RE) relays 7-byte control
 * frames (contracts/rf_protocol.h) from the laptop over the radio; this node:
 *
 *   1. brings up the clock, tick, status LED, SPI2, the nRF24 and CAN,
 *   2. reads each payload and validates magic + CRC,
 *   3. tracks the sequence counter to count dropped and duplicate frames,
 *   4. republishes the command on CAN as 0x200 Gateway_DriverCmd, and
 *   5. applies the link-loss failsafe: no valid frame for CAR_FAILSAFE_MS =>
 *      publish LinkOk = 0 with the command fields ZEROED, so a stale command
 *      can never be acted upon downstream.
 *
 * Publishes : 0x200 Gateway_DriverCmd (20 ms), 0x700 GW_Heartbeat (100 ms)
 * Consumes  : 0x300 VC_Status, 0x310 VC_Motion - held for the telemetry uplink
 *
 * All pins and RF parameters live in gateway_config.h.
 *
 * Status LED (PA5): off = not running, 2 flashes = clock, 3 = radio, 4 = CAN,
 * brief flash = waiting for frames, ~4 Hz = link up disarmed, solid = armed.
 ******************************************************************************
 */
#include "drv_common.h"
#include "drv_clock.h"
#include "drv_gpio.h"
#include "drv_spi.h"
#include "drv_timer.h"
#include "drv_systick.h"
#include "drv_can.h"
#include "nrf24.h"
#include "rf_protocol.h"
#include "gateway_config.h"
#include "gw_debug.h"

#include "adas.h"           /* generated from contracts/adas.dbc */
#include "e2e.h"

#include <string.h>

#define forever for (;;)

/* ---- Received-command state --------------------------------------------- */
static rf_control_frame_t s_latest;      /* last valid frame                 */
static uint32_t s_last_rx_ms;            /* when it arrived                  */
static bool     s_have_frame;            /* anything received yet?           */
static bool     s_radio_ok;              /* did the radio initialise?        */
static bool     s_clock_ok;              /* did the PLL reach the target?    */
static bool     s_can_ok;
/* Captured ONCE at start-up, before anything can clear it. Published in the
 * heartbeat so a node that reset mid-run says so on the bus - the difference
 * between "the link dropped" and "the node rebooted" is invisible otherwise,
 * and they need completely different fixes. */
static DRV_ResetReason s_reset_reason;
                /* did CAN leave init mode?         */

/* ---- Sequence tracking --------------------------------------------------- */
static uint8_t  s_last_seq;
static bool     s_seq_valid;
static uint32_t s_dropped;               /* frames lost in transit           */
static uint32_t s_duplicates;            /* retransmits we already applied   */

/* Valid frames in the last 16 expected slots, published as GW_LinkQuality. */
static uint16_t s_rx_window;

/* ---- CAN state ----------------------------------------------------------- */
static uint8_t  s_alv_cmd;               /* alive counter for 0x200          */
static uint8_t  s_alv_hb;                /* alive counter for 0x700          */
static uint32_t s_next_cmd_ms;
static uint32_t s_next_hb_ms;
static uint32_t s_tx_dropped;

/* Latest telemetry from the vehicle node. Not yet forwarded up the radio link
 * - kept because the station's TFT is the next consumer, and because a stale
 * value here is the cheapest evidence that the vehicle node has gone quiet. */
static struct adas_vc_status_t s_vc_status;
static struct adas_vc_motion_t s_vc_motion;

static uint32_t s_last_vc_ms;
static E2E_Receiver s_rx_status;
static E2E_Receiver s_rx_motion;
static E2E_Receiver s_rx_vc_hb;
static E2E_Receiver s_rx_sf_hb;
static E2E_Receiver s_rx_sf_obj;
static uint32_t s_e2e_errors;

/* Latest fused object from the sensor node (0x100) - the AEB's own input, and
 * the only range figure that reaches the operator. Tracked separately from the
 * SF heartbeat: a node can be alive and heartbeating while its whole array has
 * failed, so heartbeat presence must NOT be read as "the range is good". */
static struct adas_sensor_front_object_t s_sf_object;
static uint32_t s_last_sf_obj_ms;
static bool     s_seen_sf_obj;

/* ---- live SWD view (gw_debug.h) ------------------------------------------
 * Counters that exist only to be read by a debugger. Kept next to the state
 * they describe rather than hidden away, because a counter nobody can see is
 * how the last three faults stayed invisible for so long. */
volatile gw_debug_t g_gw_debug;
static uint32_t s_loop_passes;
static uint32_t s_rf_frames;
static uint32_t s_can_rx_status;
static uint32_t s_can_rx_motion;
static uint32_t s_can_rx_object;
static uint32_t s_link_resyncs;   /* times the link came back after dying */
static uint32_t s_next_dbg_ms;

/* Heartbeat-derived node presence, and the diagnostics the telemetry frame
 * rotates through one byte at a time. */
static uint32_t s_last_vc_hb_ms;
static uint32_t s_last_sf_hb_ms;
static bool     s_seen_vc_hb;
static bool     s_seen_sf_hb;
static uint8_t  s_vc_can_tec;
static uint8_t  s_vc_can_rec;
static uint16_t s_vc_uptime;
static uint8_t  s_slow_id;       /* next rf_slow_id_t to send */

/* -------------------------------------------------------------------------- */
/*  nRF24 hardware hooks                                                      */
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
/*  Status LED                                                                */
/* -------------------------------------------------------------------------- */

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

/** @brief True while the fused object list is arriving on time.
 *
 * Separate from SF heartbeat presence on purpose - see the note on
 * s_sf_object. Declared here so status/telemetry share one definition of
 * "the range is current". */
static bool sf_object_ok(void);

/** @brief True while frames are arriving on time. */
static bool link_ok(void)
{
    return s_have_frame && !DRV_SysTick_Elapsed(s_last_rx_ms, CAR_FAILSAFE_MS);
}

/** @brief @p count short flashes, then a long gap, repeating. */
static bool blink_code(uint32_t ms, uint32_t count)
{
    const uint32_t on = 150u, off = 150u, gap = 900u;
    const uint32_t burst = count * (on + off);
    const uint32_t t = ms % (burst + gap);
    return (t < burst) && ((t % (on + off)) < on);
}

static void status_led_update(void)
{
    const uint32_t ms = DRV_SysTick_GetTick();
    bool on;

    if (!s_clock_ok) {
        on = blink_code(ms, 2u);
    } else if (!s_radio_ok) {
        on = blink_code(ms, 3u);
    } else if (!s_can_ok) {
        on = blink_code(ms, 4u);
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
/*  CAN publish / consume                                                     */
/* -------------------------------------------------------------------------- */

static void can_publish(uint32_t id, uint8_t *data, uint8_t *counter)
{
    CAN_TxHeader_t frame;

    /* Order matters: pack() leaves the CRC and counter fields zero, then
     * E2E_Protect fills them. Protect touches only byte 0 and the LOW nibble of
     * byte 1, leaving the high nibble to the message. */
    E2E_Protect((uint16_t)id, data, 8u, counter);

    frame.std_id = id;
    frame.dlc    = 8u;
    memcpy(frame.data, data, 8u);

    if (!DRV_CAN_Transmit(&frame)) {
        s_tx_dropped++;      /* mailboxes full - NART means it is not retried */
    }
}

/** @brief How many of the last 16 expected frames arrived. */
static uint8_t link_quality(void)
{
    uint8_t n = 0u;
    for (uint8_t i = 0u; i < 16u; i++) {
        if (s_rx_window & (1u << i)) {
            n++;
        }
    }

    /* SATURATE AT 15. A 16-slot window yields 0..16, which is seventeen values
     * and does not fit the four bits both consumers give it - GW_LinkQuality on
     * 0x200 and the low nibble of the RF telemetry health byte.
     *
     * Unsaturated, a PERFECT link counts 16, and 16 & 0x0F is 0. The indicator
     * therefore climbed as the window filled and then vanished at the exact
     * moment the link became flawless, which reads as a dead link and is the
     * most misleading failure this signal could possibly have.
     *
     * Losing the distinction between 15-of-16 and 16-of-16 costs nothing. */
    return (n > 15u) ? 15u : n;
}

/**
 * @brief Publish 0x200 - driver intent, a straight translation of the RF frame.
 *
 * On link loss the command fields are ZEROED rather than held. A receiver that
 * only checked LinkOk and forgot to zero them itself would otherwise keep
 * driving on the last throttle it saw.
 */
static void publish_driver_cmd(uint32_t now_ms)
{
    struct adas_gateway_driver_cmd_t cmd = {0};
    uint8_t data[8];
    const bool up = link_ok();

    cmd.gw_link_ok      = up ? 1u : 0u;
    cmd.gw_link_quality = link_quality();

    if (up) {
        cmd.gw_steering  = s_latest.steering;
        cmd.gw_throttle  = s_latest.throttle;
        cmd.gw_brake     = s_latest.brake;
        cmd.gw_reverse   = rf_control_is_reverse(&s_latest) ? 1u : 0u;
        cmd.gw_armed     = rf_control_is_armed(&s_latest) ? 1u : 0u;
        cmd.gw_rf_seq    = s_latest.seq;

        /* Age of the RF frame when it goes onto CAN - one half of the
         * end-to-end latency the validation protocol has to report. */
        const uint32_t age = now_ms - s_last_rx_ms;
        cmd.gw_frame_age = (uint8_t)((age > 255u) ? 255u : age);
    }

    if (adas_gateway_driver_cmd_pack(data, &cmd, sizeof(data)) == 8) {
        can_publish(ADAS_GATEWAY_DRIVER_CMD_FRAME_ID, data, &s_alv_cmd);
    }
}

static void publish_heartbeat(uint32_t now_ms)
{
    struct adas_gw_heartbeat_t hb = {0};
    uint8_t data[8];
    uint8_t tec = 0u, rec = 0u;

    DRV_CAN_GetErrorCounters(&tec, &rec);

    hb.hbgw_node_id      = CAR_NODE_ID_GATEWAY;
    hb.hbgw_node_state   = DRV_CAN_IsBusOff()
                             ? ADAS_GW_HEARTBEAT_HBGW_NODE_STATE_FAULT_CHOICE
                             : (link_ok()
                                 ? ADAS_GW_HEARTBEAT_HBGW_NODE_STATE_RUN_CHOICE
                                 : ADAS_GW_HEARTBEAT_HBGW_NODE_STATE_DEGRADED_CHOICE);
    hb.hbgw_reset_reason = (uint8_t)s_reset_reason;
    hb.hbgw_uptime       = (uint16_t)(now_ms / 1000u);
    hb.hbgw_can_tec      = tec;
    hb.hbgw_can_rec      = rec;

    if (adas_gw_heartbeat_pack(data, &hb, sizeof(data)) == 8) {
        can_publish(ADAS_GW_HEARTBEAT_FRAME_ID, data, &s_alv_hb);
    }
}

/** @brief Drain the receive FIFO: vehicle telemetry for the uplink. */
static void can_poll(uint32_t now_ms)
{
    CAN_RxHeader_t rx;

    while (DRV_CAN_Receive(&rx)) {
        if (rx.dlc != 8u) {
            continue;
        }

        /* E2E_LOST means earlier frames vanished but THIS one is CRC-valid and
         * fresh, so it is used. Only CRC_ERROR and REPEATED are discarded -
         * see the fuller note in car/nodes/vehicle_control/main.c. */
        E2E_Status st;

        switch (rx.std_id) {
        case ADAS_VC_STATUS_FRAME_ID:
            st = E2E_Check((uint16_t)rx.std_id, rx.data, 8u, &s_rx_status);
            if ((st == E2E_OK) || (st == E2E_LOST)) {
                s_can_rx_status++;
                (void)adas_vc_status_unpack(&s_vc_status, rx.data, 8u);
                s_last_vc_ms = now_ms;
            }
            if (st != E2E_OK) {
                s_e2e_errors++;
            }
            break;

        case ADAS_VC_MOTION_FRAME_ID:
            st = E2E_Check((uint16_t)rx.std_id, rx.data, 8u, &s_rx_motion);
            if ((st == E2E_OK) || (st == E2E_LOST)) {
                s_can_rx_motion++;
                (void)adas_vc_motion_unpack(&s_vc_motion, rx.data, 8u);
                s_last_vc_ms = now_ms;
            }
            if (st != E2E_OK) {
                s_e2e_errors++;
            }
            break;

        case ADAS_VC_HEARTBEAT_FRAME_ID: {
            st = E2E_Check((uint16_t)rx.std_id, rx.data, 8u, &s_rx_vc_hb);
            if ((st == E2E_OK) || (st == E2E_LOST)) {
                struct adas_vc_heartbeat_t hb;
                (void)adas_vc_heartbeat_unpack(&hb, rx.data, 8u);
                s_vc_can_tec    = (uint8_t)hb.hbvc_can_tec;
                s_vc_can_rec    = (uint8_t)hb.hbvc_can_rec;
                s_vc_uptime     = (uint16_t)hb.hbvc_uptime;
                s_last_vc_hb_ms = now_ms;
                s_seen_vc_hb    = true;
            }
            if (st != E2E_OK) {
                s_e2e_errors++;
            }
            break;
        }

        case ADAS_SENSOR_FRONT_OBJECT_FRAME_ID:
            st = E2E_Check((uint16_t)rx.std_id, rx.data, 8u, &s_rx_sf_obj);
            if ((st == E2E_OK) || (st == E2E_LOST)) {
                s_can_rx_object++;
                (void)adas_sensor_front_object_unpack(&s_sf_object, rx.data, 8u);
                s_last_sf_obj_ms = now_ms;
                s_seen_sf_obj    = true;
            }
            if (st != E2E_OK) {
                s_e2e_errors++;
            }
            break;

        case ADAS_SF_HEARTBEAT_FRAME_ID:
            st = E2E_Check((uint16_t)rx.std_id, rx.data, 8u, &s_rx_sf_hb);
            if ((st == E2E_OK) || (st == E2E_LOST)) {
                s_last_sf_hb_ms = now_ms;
                s_seen_sf_hb    = true;
            }
            if (st != E2E_OK) {
                s_e2e_errors++;
            }
            break;

        default:
            break;   /* not ours - the hardware filter should have caught it */
        }
    }
}

/* -------------------------------------------------------------------------- */
/*  Telemetry uplink (car -> station)                                         */
/* -------------------------------------------------------------------------- */

/* A node counts as present if its heartbeat arrived within three cycles. */
#define NODE_TIMEOUT_MS  (3u * ADAS_VC_HEARTBEAT_CYCLE_TIME_MS)

/* 0x100 is event-triggered, published whenever any element reports, so it
 * arrives at roughly its 13 ms cycle time but not on a fixed schedule. Five
 * cycles rides out the jitter and a missed round without letting a range the
 * operator is steering by go stale unnoticed.
 *
 * Deliberately well under the sensor node's own 150 ms staleness sweep: if an
 * element dies, that node drops it from the fusion and keeps publishing, so
 * this timeout should only ever fire when 0x100 stops ENTIRELY. */
#define SF_OBJECT_TIMEOUT_MS  (5u * ADAS_SENSOR_FRONT_OBJECT_CYCLE_TIME_MS)

static bool sf_object_ok(void)
{
    return s_seen_sf_obj &&
           !DRV_SysTick_Elapsed(s_last_sf_obj_ms, SF_OBJECT_TIMEOUT_MS);
}

/** @brief One diagnostic byte per frame, cycling through them all in ~1 s. */
static uint8_t slow_value(uint8_t id)
{
    switch (id) {
    case RF_SLOW_VC_UPTIME_LO:  return (uint8_t)(s_vc_uptime & 0xFFu);
    case RF_SLOW_VC_UPTIME_HI:  return (uint8_t)(s_vc_uptime >> 8);
    case RF_SLOW_VC_CAN_TEC:    return s_vc_can_tec;
    case RF_SLOW_VC_CAN_REC:    return s_vc_can_rec;
    case RF_SLOW_VC_E2E_ERRORS: return (uint8_t)s_vc_status.vc_e2e_err_count;
    case RF_SLOW_GW_CAN_TEC: {
        uint8_t tec = 0u;
        DRV_CAN_GetErrorCounters(&tec, NULL);
        return tec;
    }
    case RF_SLOW_GW_CAN_REC: {
        uint8_t rec = 0u;
        DRV_CAN_GetErrorCounters(NULL, &rec);
        return rec;
    }
    case RF_SLOW_GW_RF_DROPPED:
        return (uint8_t)((s_dropped > 255u) ? 255u : s_dropped);
    default:
        return 0u;
    }
}

/**
 * @brief Queue the telemetry that will ride out on the next auto-ACK.
 *
 * Called once per RECEIVED control frame, which is what keeps exactly one
 * payload queued: the chip holds three, and queueing faster than packets
 * arrive would mean the station reads data that is several frames stale.
 *
 * Everything here comes from CAN messages this node already receives - the
 * gateway measures nothing itself except the RF link quality.
 */
static void telemetry_queue(uint8_t seq_echo)
{
    rf_telemetry_frame_t t;
    const bool vc_present = s_seen_vc_hb &&
                            !DRV_SysTick_Elapsed(s_last_vc_hb_ms, NODE_TIMEOUT_MS);
    const bool sf_present = s_seen_sf_hb &&
                            !DRV_SysTick_Elapsed(s_last_sf_hb_ms, NODE_TIMEOUT_MS);

    memset(&t, 0, sizeof(t));
    t.seq_echo = seq_echo;

    if (vc_present) {
        t.state = rf_telem_pack_state((uint8_t)s_vc_status.vc_vehicle_state,
                                      (uint8_t)s_vc_status.vc_aeb_state,
                                      s_vc_status.vc_degraded != 0u);
        t.faults   = (uint8_t)s_vc_status.vc_faults;
        t.throttle = (uint8_t)s_vc_status.vc_applied_throttle;
        t.brake    = (uint8_t)s_vc_status.vc_applied_brake;
        t.speed    = (int16_t)s_vc_motion.vcm_speed;
    } else {
        /* Say nothing rather than something stale: a display showing the last
         * known state of a node that has gone silent is worse than a blank. */
        t.state = rf_telem_pack_state(0u, 0u, true);
    }

    /* Range comes from the FUSED object (0x100), not from any one element: it
     * is the closest valid target across the whole front array - the same
     * figure the AEB reasons about. Showing the operator anything else would
     * mean the screen and the braking decision disagree.
     *
     * Three separate things must hold before a number is shown, because the
     * uplink has one uint16 and no status field beside it - so everything that
     * is not a trustworthy distance has to collapse into the sentinel:
     *
     *   1. a 0x100 has actually arrived, and recently;
     *   2. the sensor node itself calls it VALID - NO_TARGET, DEGRADED and
     *      FAULT all mean "do not draw a number";
     *   3. the value cannot alias the sentinel.
     *
     * Note this is independent of sf_present: the heartbeat says the node is
     * alive, which it still is when every element has failed. */
    if (sf_object_ok() &&
        (s_sf_object.sf_status ==
         ADAS_SENSOR_FRONT_OBJECT_SF_STATUS_VALID_CHOICE)) {
        uint16_t mm = (uint16_t)s_sf_object.sf_range;

        /* 0xFFFF is the "no range" sentinel, so a real reading must never be
         * able to reach it. SF_Range is declared [0|4000] mm in the DBC and
         * the sensor node gates at 2000, so this cannot trip today - but the
         * clamp costs nothing and the alternative is a maximum-range target
         * silently rendering as "---". */
        t.range = (mm >= RF_TELEM_RANGE_NONE) ? (RF_TELEM_RANGE_NONE - 1u) : mm;
    } else {
        t.range = RF_TELEM_RANGE_NONE;
    }

    t.health = (uint8_t)(link_quality() & RF_TELEM_LINKQ_MASK);
    t.health |= RF_TELEM_NODE_GW;                       /* we are, by definition */
    if (vc_present) { t.health |= RF_TELEM_NODE_VC; }
    if (sf_present) { t.health |= RF_TELEM_NODE_SF; }
    if (s_e2e_errors != 0u) { t.health |= RF_TELEM_E2E_ERROR; }

    t.slow_id  = s_slow_id;
    t.slow_val = slow_value(s_slow_id);
    s_slow_id  = (uint8_t)((s_slow_id + 1u) % (uint8_t)RF_SLOW_COUNT);

    rf_telemetry_frame_finalize(&t);

    (void)nrf24_write_ack_payload(&s_radio, 0u, (const uint8_t *)&t, sizeof(t));
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
        .ack_payload   = CAR_RF_ACK_PAYLOAD,
    };
    memcpy(rf_cfg.address, addr, NRF24_ADDR_WIDTH);

    s_radio_ok = nrf24_init(&s_radio, &s_nrf_hal, &rf_cfg);
    if (s_radio_ok) {
        nrf24_set_rx_mode(&s_radio);
    }
}

static void board_init(void)
{
    /* FIRST: reads and clears RCC_CSR. Anything that reconfigures the
     * clock tree must not run before this. */
    s_reset_reason = DRV_Clock_ResetReason();

    /* Keep the result: a missing or dead HSE crystal makes this time out and
     * the part carries on at 8 MHz HSI. Everything still runs, just 9x slow -
     * and every CAN bit-timing number assumes 72 MHz. */
    s_clock_ok = (DRV_Clock_Init72MHz(CAR_HSE_HZ) == DRV_OK);
    DRV_Delay_Init();
    DRV_SysTick_Init(CAR_TICK_IRQ_PRIORITY);

    /* Drive the inactive level before switching the pin to an output, so the
     * LED never flashes on reset. 2 MHz: an LED needs no slew rate. */
    DRV_GPIO_Write(CAR_LED_STATUS_PORT, CAR_LED_STATUS_PIN,
                   CAR_LED_ACTIVE_LOW ? GPIO_HIGH : GPIO_LOW);
    DRV_GPIO_Init(CAR_LED_STATUS_PORT, CAR_LED_STATUS_PIN,
                  GPIO_MODE_OUT_2M, GPIO_CNF_OUT_PP, GPIO_PULL_NONE);

    s_can_ok = DRV_CAN_Init(CAR_CAN_MODE);
    radio_init();
}

/* -------------------------------------------------------------------------- */
/*  Frame handling                                                            */
/* -------------------------------------------------------------------------- */

/* Update the drop/duplicate statistics. Returns false for a duplicate. */
static bool sequence_track(uint8_t seq)
{
    if (!s_seq_valid) {
        s_last_seq  = seq;
        s_seq_valid = true;
        return true;
    }

    /* RESYNC, don't count. The sequence number is 8 bits at a 20 ms cycle, so
     * it wraps every ~5.1 s. Any outage longer than that makes the gap
     * ALIASED - an arbitrary 0..254 that says nothing about how many frames
     * were really lost. Adding it to s_dropped does not just lose precision,
     * it invents traffic: one link loss can book 250 drops that never
     * happened, and after a few of those the counter is pure noise and can no
     * longer show a genuinely degrading link.
     *
     * So a frame arriving while the link is DOWN re-establishes the baseline
     * and is counted as a link loss instead. s_dropped then means only what it
     * can actually measure: frames missing from an otherwise live link. */
    if (!link_ok()) {
        s_link_resyncs++;
        s_last_seq = seq;
        return true;
    }

    /* Unsigned subtraction is wrap-safe across the 255 -> 0 rollover. */
    const uint8_t gap = (uint8_t)(seq - s_last_seq);
    if (gap == 0u) {
        s_duplicates++;
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
        s_rf_frames++;
        s_rx_window  = (uint16_t)((s_rx_window << 1) | 1u);

        (void)sequence_track(frame.seq);

        /* Reload the ACK payload for the NEXT control frame. Doing it here -
         * one queue per packet received - is what keeps the station reading
         * fresh data rather than the oldest of three queued frames. */
        telemetry_queue(frame.seq);
    }
}

/**
 * @brief Republish the node state into g_gw_debug for a debugger to read.
 *
 * Rate limited: the nRF24 register reads below are real SPI traffic, and the
 * main loop spins far faster than any human reads a diagnostic. `seq` still
 * carries the FREE-RUNNING pass count, so a stalled loop shows up even though
 * this function runs only every 50 ms.
 *
 * The radio registers are read LIVE rather than cached from bring-up. That is
 * the whole point of doing this in the real firmware: a module that dies or a
 * wire that falls out mid-run changes these, while radio_ok stays true forever.
 */
static void gw_debug_update(uint32_t now_ms)
{
    if ((int32_t)(now_ms - s_next_dbg_ms) < 0) {
        return;
    }
    s_next_dbg_ms = now_ms + 50u;

    g_gw_debug.seq       = s_loop_passes;
    g_gw_debug.uptime_ms = now_ms;

    g_gw_debug.clock_ok = s_clock_ok ? 1u : 0u;
    g_gw_debug.can_ok   = s_can_ok   ? 1u : 0u;
    g_gw_debug.radio_ok = s_radio_ok ? 1u : 0u;

    g_gw_debug.nrf_config = nrf24_read_register(&s_radio, NRF24_REG_CONFIG);
    g_gw_debug.nrf_status = nrf24_read_register(&s_radio, NRF24_REG_STATUS);
    g_gw_debug.nrf_rf_ch  = nrf24_read_register(&s_radio, NRF24_REG_RF_CH);
    g_gw_debug.nrf_fifo   = nrf24_read_register(&s_radio,
                                                NRF24_REG_FIFO_STATUS);

    g_gw_debug.rf_frames     = s_rf_frames;
    g_gw_debug.rf_dropped    = s_dropped;
    g_gw_debug.rf_duplicates = s_duplicates;
    g_gw_debug.rf_age_ms     = s_have_frame ? (now_ms - s_last_rx_ms) : 0xFFFFFFFFu;
    g_gw_debug.link_ok       = link_ok() ? 1u : 0u;
    g_gw_debug.link_quality  = link_quality();

    g_gw_debug.can_rx_status  = s_can_rx_status;
    g_gw_debug.can_rx_motion  = s_can_rx_motion;
    g_gw_debug.can_tx_dropped = s_tx_dropped;
    g_gw_debug.e2e_errors     = s_e2e_errors;

    uint8_t tec = 0u, rec = 0u;
    DRV_CAN_GetErrorCounters(&tec, &rec);
    g_gw_debug.can_tec     = tec;
    g_gw_debug.can_rec     = rec;
    g_gw_debug.can_bus_off = DRV_CAN_IsBusOff() ? 1u : 0u;

    g_gw_debug.seen_vc  = s_seen_vc_hb ? 1u : 0u;
    g_gw_debug.seen_sf  = s_seen_sf_hb ? 1u : 0u;
    g_gw_debug.vc_state = (uint8_t)s_vc_status.vc_vehicle_state;

    /* Appended at the END of the struct, not inserted: read_gateway.cmake
     * decodes by word index, so inserting would silently shift every field
     * after it. */
    g_gw_debug.can_rx_object = s_can_rx_object;
    g_gw_debug.link_resyncs  = s_link_resyncs;
    g_gw_debug.sf_range      = (uint16_t)s_sf_object.sf_range;
    g_gw_debug.sf_status     = (uint8_t)s_sf_object.sf_status;
    g_gw_debug.sf_obj_fresh  = sf_object_ok() ? 1u : 0u;

    g_gw_debug.magic = GW_DEBUG_MAGIC;   /* last: the struct is now valid */
}

int main(void)
{
    board_init();

    s_next_cmd_ms = DRV_SysTick_GetTick();
    s_next_hb_ms  = s_next_cmd_ms;

    forever {
        const uint32_t now = DRV_SysTick_GetTick();

        if (s_radio_ok) {
            radio_poll();
        }
        if (s_can_ok) {
            can_poll(now);

            /* Cyclic, NOT event-triggered. A receiver times out on absence, so
             * the command must keep flowing even when the radio has gone quiet
             * - that is exactly when LinkOk = 0 needs to be heard. */
            if ((int32_t)(now - s_next_cmd_ms) >= 0) {
                publish_driver_cmd(now);
                s_next_cmd_ms += CAR_CAN_CMD_PERIOD_MS;
                if (!link_ok()) {
                    s_rx_window <<= 1;   /* an expected slot went unfilled */
                }
            }
            if ((int32_t)(now - s_next_hb_ms) >= 0) {
                publish_heartbeat(now);
                s_next_hb_ms += CAR_CAN_HEARTBEAT_MS;
            }
        }

        status_led_update();

        s_loop_passes++;
        gw_debug_update(now);
    }
}
