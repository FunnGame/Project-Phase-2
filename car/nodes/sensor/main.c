/**
 ******************************************************************************
 * @file    main.c
 * @brief   VL53L0X sensor node - publishes object lists on the CAN bus.
 *
 * Ranges every VL53L0X on the node and publishes two messages from
 * contracts/adas.dbc, so a USB-CAN adapter decodes them by name instead of the
 * ad-hoc UART text this file used to print:
 *
 *   0x100 SensorFront_Object  the FUSED closest in-path object - the AEB input.
 *                             Event-triggered: sent whenever any sensor reports,
 *                             so it is never staler than one measurement.
 *   0x101 SensorFront_Raw     per-sensor ranges, round-robin, one sensor per
 *                             slot. Diagnostics; not the AEB path.
 *   0x702 SF_Heartbeat        liveness, uptime and CAN error counters, 100 ms.
 *
 * Every frame carries E2E protection (shared/e2e): CRC-8 over the CAN
 * identifier plus payload, and a 4-bit alive counter.
 *
 * FOUR VL53L0X share one I2C bus, indexed 0..3 left to right across the front
 * of the car. They all boot at 0x29, so HAL_VL53_Init() wakes them one at a
 * time via their XSHUT lines and readdresses each before the next.
 *
 * Wiring (all on car/drivers):
 *   VL53L0X : I2C1 SCL PB6 / SDA PB7
 *             XSHUT PB0 (FL) / PB1 (CL) / PB10 (CR) / PB11 (FR)
 *   CAN     : PB8 RX / PB9 TX, 500 kbit/s, via a transceiver
 *   Status  : PC13 on-board LED
 *
 * There is no UART any more, so start-up failures are LED blink codes - see
 * SENSOR_BLINK_* in sensor_config.h.
 *
 * Build target: `sensor` (see CMakeLists.txt).
 ******************************************************************************
 */
#include "drv_clock.h"
#include "drv_timer.h"      /* DRV_Delay_Init / DRV_Delay_Ms                  */
#include "drv_systick.h"    /* 1 ms tick - leaves every TIMx free             */
#include "drv_i2c.h"
#include "drv_can.h"
#include "drv_gpio.h"

#include "hal_vl53.h"       /* also pulls in types.h: Status_t, uint8, uint16 */
#include "sensor_config.h"

#include "adas.h"           /* generated from contracts/adas.dbc              */
#include "e2e.h"

/* Signed range rate saturates at the DBC's declared limits. */
#define RATE_LIMIT_MMPS     8000

/* ===== Node state ========================================================= */

/** Per-sensor measurement, updated as readings arrive. */
typedef struct {
    uint16_t range_mm;      /**< Latest distance.                             */
    int16_t  rate_mmps;     /**< First difference. NEGATIVE means closing.    */
    uint16_t prev_mm;       /**< Previous distance, for the difference.       */
    uint32_t prev_ms;       /**< When prev_mm was taken.                      */
    uint32_t stamp_ms;      /**< When range_mm was taken - drives SlotAge.    */
    uint8_t  track_age;     /**< Consecutive cycles with a valid target.      */
    uint8_t  status;        /**< ADAS_..._SF_STATUS_*_CHOICE                  */
    bool     seeded;        /**< prev_mm is meaningful.                       */
} SensorState;

static SensorState s_sensor[VL53_COUNT];

/* One alive counter per message - they are independent streams. */
static uint8_t s_alv_object;
static uint8_t s_alv_raw;
static uint8_t s_alv_hb;

static uint32_t s_tx_dropped;
/* Captured ONCE at start-up, before anything can clear it. Published in the
 * heartbeat so a node that reset mid-run says so on the bus - the difference
 * between "the link dropped" and "the node rebooted" is invisible otherwise,
 * and they need completely different fixes. */
static DRV_ResetReason s_reset_reason;
   /* frames lost to full mailboxes */

/* ===== Status LED ========================================================= */

static void led_set(bool on)
{
#if SENSOR_LED_ACTIVE_LOW
    DRV_GPIO_Write(SENSOR_LED_PORT, SENSOR_LED_PIN, on ? GPIO_LOW : GPIO_HIGH);
#else
    DRV_GPIO_Write(SENSOR_LED_PORT, SENSOR_LED_PIN, on ? GPIO_HIGH : GPIO_LOW);
#endif
}

/** @brief Never returns: flash @p code times, pause, repeat.
 *
 * The only way to report a failure once the UART is gone. Deliberately busy -
 * a node that cannot initialise must not look alive on the bus. */
static void fatal(uint32_t code)
{
    for (;;) {
        for (uint32_t i = 0u; i < code; i++) {
            led_set(true);
            DRV_Delay_Ms(200u);
            led_set(false);
            DRV_Delay_Ms(200u);
        }
        DRV_Delay_Ms(1200u);
    }
}

/* ===== CAN publish helpers ================================================ */

/**
 * @brief E2E-stamp a packed frame and hand it to the driver.
 *
 * Order matters: the generated pack() writes the payload with the CRC and
 * counter fields zeroed, then E2E_Protect() fills them in. Protect touches only
 * byte 0 and the LOW nibble of byte 1, which is why the high nibble is free for
 * SF_ObjCount / SFR_SensorIdx / HBSF_NodeId.
 */
static void can_publish(uint32_t id, uint8_t *data, uint8_t *counter)
{
    CAN_TxHeader_t frame;

    E2E_Protect((uint16_t)id, data, 8u, counter);

    frame.std_id = id;
    frame.dlc    = 8u;
    for (uint8_t i = 0u; i < 8u; i++) {
        frame.data[i] = data[i];
    }

    if (!DRV_CAN_Transmit(&frame)) {
        s_tx_dropped++;   /* mailboxes full - NART means it is not retried */
    }
}

/**
 * @brief Publish 0x100: the closest valid object across the whole array.
 *
 * Selection is a plain minimum over the four ranges - no weighting by bearing,
 * no gating on which elements are looking down the vehicle's path. With a
 * narrow four-element array that is a reasonable AEB input, but it does mean a
 * wall the outer sensor clips at an angle counts the same as an obstacle dead
 * ahead. Bearing-aware fusion belongs in car/lib/tracker, which does not exist
 * yet; this is the honest placeholder until it does.
 */
static void publish_object(uint32_t now_ms)
{
    struct adas_sensor_front_object_t obj = {0};
    uint8_t data[8];
    const SensorState *best = NULL;
    uint8_t valid = 0u;
    uint32_t age_ms;

    for (uint8_t i = 0u; i < (uint8_t)VL53_COUNT; i++) {
        if (s_sensor[i].status != ADAS_SENSOR_FRONT_OBJECT_SF_STATUS_VALID_CHOICE) {
            continue;
        }
        valid++;
        if ((best == NULL) || (s_sensor[i].range_mm < best->range_mm)) {
            best = &s_sensor[i];
        }
    }

    /* SF_ObjCount is two bits, so four valid sensors report as 3. That is a
     * deliberate limit of the contract, not a bug here: the field exists to say
     * "how much of the array agrees there is something there", and 3+ is as
     * useful an answer as 4. Widening it would be a DBC change. */
    obj.sf_obj_count = (valid > 3u) ? 3u : valid;

    if (best != NULL) {
        age_ms = now_ms - best->stamp_ms;
        obj.sf_range      = best->range_mm;
        obj.sf_range_rate = best->rate_mmps;
        obj.sf_track_age  = best->track_age;
        obj.sf_status     = ADAS_SENSOR_FRONT_OBJECT_SF_STATUS_VALID_CHOICE;
        obj.sf_confidence = 58u;   /* until the tracker supplies a real score */
        /* SF_SampleAge is 2 bits at 20 ms per step, so the struct field wants
         * 0..3 - not milliseconds. Saturate in PHYSICAL units first, because
         * _encode() only divides: it does not clamp, and a raw value above 3
         * is masked away by pack() rather than pinned to the maximum. An age
         * of 80 ms would otherwise wrap round and transmit as "0 ms - fresh",
         * which is the one direction this field must never lie in.
         *
         * Truncation means the result rounds DOWN to the step below, so treat
         * SF_SampleAge as a lower bound on staleness. */
        const uint32_t capped_ms = (age_ms > 60u) ? 60u : age_ms;
        obj.sf_sample_age =
            adas_sensor_front_object_sf_sample_age_encode((double)capped_ms);
    } else {
        obj.sf_status = ADAS_SENSOR_FRONT_OBJECT_SF_STATUS_NO_TARGET_CHOICE;
    }

    if (adas_sensor_front_object_pack(data, &obj, sizeof(data)) == 8) {
        can_publish(ADAS_SENSOR_FRONT_OBJECT_FRAME_ID, data, &s_alv_object);
    }
}

/** @brief Publish 0x101 for one sensor - the round-robin slot. */
static void publish_raw(uint8_t idx, uint32_t now_ms)
{
    struct adas_sensor_front_raw_t raw = {0};
    uint8_t data[8];
    uint32_t age_ms = now_ms - s_sensor[idx].stamp_ms;

    raw.sfr_sensor_idx = idx;
    raw.sfr_range      = s_sensor[idx].range_mm;
    raw.sfr_range_rate = s_sensor[idx].rate_mmps;
    raw.sfr_status     = s_sensor[idx].status;
    raw.sfr_confidence = (s_sensor[idx].status ==
                          ADAS_SENSOR_FRONT_RAW_SFR_STATUS_VALID_CHOICE) ? 58u : 0u;
    raw.sfr_slot_age   = (uint8_t)((age_ms > 255u) ? 255u : age_ms);

    if (adas_sensor_front_raw_pack(data, &raw, sizeof(data)) == 8) {
        can_publish(ADAS_SENSOR_FRONT_RAW_FRAME_ID, data, &s_alv_raw);
    }
}

/** @brief Publish 0x702: liveness, uptime and real CAN error counters. */
static void publish_heartbeat(uint32_t now_ms)
{
    struct adas_sf_heartbeat_t hb = {0};
    uint8_t data[8];
    uint8_t tec = 0u;
    uint8_t rec = 0u;

    DRV_CAN_GetErrorCounters(&tec, &rec);

    hb.hbsf_node_id      = SENSOR_NODE_ID;
    hb.hbsf_node_state   = DRV_CAN_IsBusOff()
                             ? ADAS_SF_HEARTBEAT_HBSF_NODE_STATE_FAULT_CHOICE
                             : ADAS_SF_HEARTBEAT_HBSF_NODE_STATE_RUN_CHOICE;
    hb.hbsf_reset_reason = (uint8_t)s_reset_reason;
    hb.hbsf_uptime       = (uint16_t)(now_ms / 1000u);
    hb.hbsf_can_tec      = tec;
    hb.hbsf_can_rec      = rec;

    if (adas_sf_heartbeat_pack(data, &hb, sizeof(data)) == 8) {
        can_publish(ADAS_SF_HEARTBEAT_FRAME_ID, data, &s_alv_hb);
    }
}

/* ===== Measurement ======================================================== */

/** @brief Drop a sensor's track without declaring the sensor itself bad.
 *
 * The sensor answered; it just has nothing believable to report. Note
 * stamp_ms is still refreshed - the element is alive, so the staleness sweep
 * must not also mark it FAULT. */
static void sensor_no_target(SensorState *s, uint32_t now_ms)
{
    s->status    = ADAS_SENSOR_FRONT_OBJECT_SF_STATUS_NO_TARGET_CHOICE;
    s->range_mm  = 0u;
    s->rate_mmps = 0;
    s->track_age = 0u;
    s->seeded    = false;
    s->stamp_ms  = now_ms;
}

/**
 * @brief Fold a new reading into a sensor's state, deriving the range rate.
 *
 * @param range_status the ST API's verdict (VL53_RANGESTATUS_*). This is not
 *        advisory: RangeMilliMeter holds a number on every failure path, and
 *        an empty scene produces ~8190 mm - four times what this part can
 *        measure - with RangeStatus = PHASE. Trusting the millimetres alone
 *        publishes that as a valid target.
 */
static void sensor_update(uint8_t idx, uint16_t mm, uint8_t range_status,
                          uint32_t now_ms)
{
    SensorState *s = &s_sensor[idx];
    uint32_t dt_ms;
    int32_t rate;

    /* A hardware fail is the sensor itself, not the scene: the element is
     * broken and must stop contributing to the fused object entirely, rather
     * than merely reporting nothing this cycle. */
    if (range_status == VL53_RANGESTATUS_HW) {
        s->status    = ADAS_SENSOR_FRONT_OBJECT_SF_STATUS_FAULT_CHOICE;
        s->range_mm  = 0u;
        s->rate_mmps = 0;
        s->track_age = 0u;
        s->seeded    = false;
        s->stamp_ms  = now_ms;
        return;
    }

    /* Everything else the API rejected - sigma, weak signal, phase, min range,
     * no-update - means "nothing believable in view". So does a reading that
     * survived the API but lies outside what the configured ranging profile
     * can physically reach. Both become NO_TARGET rather than a target at a
     * fictitious distance. */
    if ((range_status != VL53_RANGESTATUS_VALID) ||
        (mm < VL53_MIN_VALID_MM) || (mm > VL53_MAX_VALID_MM)) {
        sensor_no_target(s, now_ms);
        return;
    }

    if (s->seeded) {
        dt_ms = now_ms - s->prev_ms;
        if (dt_ms > 0u) {
            /* Negative = closing, the sign convention AEB depends on.
             * A plain first difference: this is bring-up instrumentation, not
             * the tracker that car/lib/tracker will eventually provide. */
            rate = (((int32_t)mm - (int32_t)s->prev_mm) * 1000) / (int32_t)dt_ms;
            if (rate > RATE_LIMIT_MMPS)  { rate = RATE_LIMIT_MMPS;  }
            if (rate < -RATE_LIMIT_MMPS) { rate = -RATE_LIMIT_MMPS; }
            s->rate_mmps = (int16_t)rate;
        }
    } else {
        s->rate_mmps = 0;
        s->seeded    = true;
    }

    s->prev_mm   = mm;
    s->prev_ms   = now_ms;
    s->range_mm  = mm;
    s->stamp_ms  = now_ms;
    s->status    = ADAS_SENSOR_FRONT_OBJECT_SF_STATUS_VALID_CHOICE;
    if (s->track_age < 255u) {
        s->track_age++;
    }
}

/**
 * @brief Drop sensors that have gone quiet.
 *
 * A sensor whose I2C stops answering never enters sensor_update() again, so
 * without this its last reading simply persists: status stays VALID, range
 * stays whatever it was, and the fused object keeps offering it to the AEB as
 * a live target. Staleness is invisible unless something looks for it.
 *
 * With one sensor that was a theoretical problem. With four it is a practical
 * one - four times the chance that exactly one of them dies while the node as
 * a whole goes on looking perfectly healthy on the bus.
 */
static void sensors_age_check(uint32_t now_ms)
{
    for (uint8_t i = 0u; i < (uint8_t)VL53_COUNT; i++) {
        if (s_sensor[i].status == ADAS_SENSOR_FRONT_OBJECT_SF_STATUS_FAULT_CHOICE) {
            continue;                       /* already known bad */
        }
        if ((now_ms - s_sensor[i].stamp_ms) > SENSOR_STALE_MS) {
            s_sensor[i].status    = ADAS_SENSOR_FRONT_OBJECT_SF_STATUS_FAULT_CHOICE;
            s_sensor[i].track_age = 0u;
            s_sensor[i].seeded    = false;
        }
    }
}

/* ===== Init =============================================================== */

/* CAN comes up before I2C so an I2C failure is still reportable - on the bus if
 * CAN is healthy, and on the LED either way. */
static void board_init(void)
{
    /* FIRST: reads and clears RCC_CSR. Anything that reconfigures the
     * clock tree must not run before this. */
    s_reset_reason = DRV_Clock_ResetReason();

    if (DRV_Clock_Init72MHz(SENSOR_HSE_HZ) != DRV_OK) {
        /* Still running, just on the 8 MHz HSI - and every CAN bit-timing
         * number assumes 72 MHz, so the bus would be unusable. */
        DRV_Delay_Init();
        DRV_GPIO_InitOutput(SENSOR_LED_PORT, SENSOR_LED_PIN, GPIO_HIGH);
        fatal(SENSOR_BLINK_CLOCK);
    }

    DRV_Delay_Init();                 /* DWT delay, needed for the VL53 boot */
    DRV_SysTick_Init(0u);             /* 1 ms tick                           */
    DRV_GPIO_InitOutput(SENSOR_LED_PORT, SENSOR_LED_PIN, GPIO_HIGH);
    led_set(false);

    if (!DRV_CAN_Init(SENSOR_CAN_MODE)) {
        /* Almost always the transceiver: unpowered, unwired, or a bus held
         * dominant. The peripheral cannot leave initialisation mode until it
         * sees 11 recessive bits. */
        fatal(SENSOR_BLINK_CAN);
    }

    const I2C_Config i2c = { .i2c = SENSOR_I2C, .speed_hz = SENSOR_I2C_HZ,
                             .remap = SENSOR_I2C_REMAP };
    if (DRV_I2C_Init(&i2c) != DRV_OK) {
        fatal(SENSOR_BLINK_I2C);
    }
}

int main(void)
{
    uint32_t round_start_ms;
    uint32_t next_hb_ms;
    uint8_t  slot = 0u;
    uint8_t  s;

    board_init();

    /* XSHUT reset-all, then enable-and-readdress one sensor at a time. */
    if (HAL_VL53_Init() != STATUS_OK) {
        fatal(SENSOR_BLINK_VL53);
    }

    /* Start the four sensors PHASED, one every PERIOD/N milliseconds, rather
     * than all together. They range concurrently either way, but staggering
     * the starts spreads their completions evenly across the 50 ms round -
     * which is what turns 0x100 into the steady ~13 ms stream the DBC
     * documents, instead of a burst of four frames every 50 ms followed by
     * silence. It also keeps each sensor's I2C read in its own slot rather
     * than four fighting for the bus at the same instant. */
    for (s = 0u; s < (uint8_t)VL53_COUNT; s++) {
        s_sensor[s].status   = ADAS_SENSOR_FRONT_OBJECT_SF_STATUS_NO_TARGET_CHOICE;
        s_sensor[s].stamp_ms = DRV_SysTick_GetTick();  /* seed the age check */
        HAL_VL53_Start(s);

        if ((s + 1u) < (uint8_t)VL53_COUNT) {
            DRV_Delay_Ms(SENSOR_MEAS_PERIOD_MS / VL53_COUNT);
        }
    }

    round_start_ms = DRV_SysTick_GetTick();
    next_hb_ms     = round_start_ms;

    for (;;) {
        uint32_t now = DRV_SysTick_GetTick();

        /* --- collect ------------------------------------------------------
         * Sweep all four, then publish the fused object ONCE if anything was
         * new. Publishing inside the loop would send up to four near-identical
         * 0x100 frames whenever two sensors happen to complete in the same
         * pass - wasted bus for a value that only changes once. */
        bool updated = false;

        for (s = 0u; s < (uint8_t)VL53_COUNT; s++) {
            uint8 ready = 0u;

            if ((HAL_VL53_IsDataReady(s, &ready) == STATUS_OK) && (ready != 0u)) {
                uint16 mm = 0u;
                uint8  rs = VL53_RANGESTATUS_NONE;

                if (HAL_VL53_ReadDistance(s, &mm, &rs) == STATUS_OK) {
                    /* STATUS_OK only means the I2C read worked. Whether the
                     * measurement is usable is rs, and sensor_update() is
                     * where that is decided. */
                    sensor_update(s, (uint16_t)mm, (uint8_t)rs, now);
                    updated = true;
                } else {
                    s_sensor[s].status =
                        ADAS_SENSOR_FRONT_OBJECT_SF_STATUS_FAULT_CHOICE;
                }
                HAL_VL53_ClearInterrupt(s);
            }
        }

        /* Ageing runs every pass, not only when something arrived - the whole
         * point is to catch the sensor that has stopped arriving. */
        sensors_age_check(now);

        if (updated) {
            publish_object(now);
        }

        /* --- round-robin raw slots ---------------------------------------
         * Slot k of N opens at (PERIOD * k) / N milliseconds into the round.
         * Integer division gives 0, 12, 25, 37 for four sensors - slot lengths
         * of 12, 13, 12, 13 ms summing to exactly 50, so the schedule stays
         * phase-locked to the measurements instead of drifting the way a fixed
         * 13 ms slot would. */
        if ((int32_t)(now - (round_start_ms +
                             (SENSOR_MEAS_PERIOD_MS * slot) / VL53_COUNT)) >= 0) {
            publish_raw(slot, now);
            slot++;
            if (slot >= (uint8_t)VL53_COUNT) {
                slot = 0u;
                round_start_ms += SENSOR_MEAS_PERIOD_MS;
            }
        }

        /* --- heartbeat ----------------------------------------------------
         * The LED flashes once per heartbeat while the bus is healthy, and
         * stays solid on bus-off - a node that is running but cannot talk. */
        if ((int32_t)(now - next_hb_ms) >= 0) {
            publish_heartbeat(now);
            next_hb_ms += SENSOR_HEARTBEAT_MS;
            led_set(true);
        } else if (!DRV_CAN_IsBusOff() &&
                   ((now - (next_hb_ms - SENSOR_HEARTBEAT_MS)) >
                    (SENSOR_HEARTBEAT_MS / 4u))) {
            led_set(false);
        }
    }
}
