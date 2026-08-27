/**
 ******************************************************************************
 * @file    vc_debug.h
 * @brief   Live + PEAK-HOLD view of the vehicle node, read over SWD.
 *
 * Exists to answer one question that nothing else can: *why did the AEB not
 * fire?* From outside the car every cause looks identical - the car simply
 * does not brake - and each guess costs a run at a wall.
 *
 * ## Why peak-hold and not a live sample
 *
 * The last time a fault on this car was chased with a live SWD dump, the
 * lesson recorded in PROJECT_LOG was that sampling `last_*` only shows the
 * instant the debugger happened to halt, which is useless for anything
 * momentary. The AEB approach is the definition of momentary: the interesting
 * cycle is the one at the closest range, and by the time the car has hit the
 * wall and you have halted it, every live value describes the aftermath.
 *
 * So the fields below latch the state AT THE CLOSEST RANGE seen while moving
 * forward, and hold it until the next run. Halt whenever you like afterwards.
 *
 * ## Reading it
 *
 *     cmake --build build --target read-vehicle
 *
 * `closest_block` is the field to look at first - it names the gate that was
 * blocking at the worst moment of the approach:
 *
 *     0 NONE       armed and not blocked. If the AEB still did not brake, the
 *                  cascade DID run - compare closest_range against closest_lpb.
 *                  range > lpb means it was never close enough to trigger, so
 *                  d_lpb is too short: CAR_AEB_DECEL_MMPS2 is optimistic.
 *     1 NO_OBJECT  0x100 stale or SF_Status not VALID. Check can_rx_object is
 *                  still climbing and VC_Faults bit 1.
 *     2 TRACK_AGE  the target never survived CAR_AEB_MIN_TRACK_AGE cycles -
 *                  readings flickering in and out of the acceptance window.
 *     3 TOO_SLOW   measured speed below CAR_AEB_MIN_SPEED_MMPS.
 *     4 OPENING    range rate read as receding.
 *     5 YAW        suppressed as a turn.
 ******************************************************************************
 */
#ifndef VC_DEBUG_H_
#define VC_DEBUG_H_

#include <stdint.h>
#include <stddef.h>

#define VC_DEBUG_MAGIC  0x56434442u   /* "VCDB" */

typedef struct {
    uint32_t magic;
    uint32_t seq;              /**< main loop passes since reset             */

    /* ---- live ------------------------------------------------------- */
    uint32_t can_rx_object;    /**< 0x100 accepted - climbing means alive    */
    uint32_t obj_age_ms;       /**< since the last accepted 0x100            */
    uint32_t e2e_errors;

    int32_t  speed_mmps;       /**< live encoder speed                       */
    int32_t  yaw_mrads;
    uint32_t range_mm;         /**< live SF_Range                            */
    int32_t  range_rate_mmps;

    uint32_t aeb_state;        /**< live VC_AebState                         */
    uint32_t aeb_block;        /**< live AEB_Block                           */
    uint32_t lpb_mm;           /**< live d_lpb                               */

    /* ---- PEAK-HOLD, latched at the closest range while moving forward ---
     * These are the run's evidence. Everything above describes the moment you
     * halted; only these describe the moment that mattered. */
    uint32_t closest_range;    /**< smallest SF_Range seen while moving      */
    int32_t  closest_speed;    /**< speed at that instant                    */
    uint32_t closest_lpb;      /**< d_lpb at that instant                    */
    uint32_t closest_block;    /**< AEB_Block at that instant - READ THIS    */
    uint32_t closest_age;      /**< SF_TrackAge at that instant              */
    uint32_t worst_state;      /**< most severe VC_AebState reached this run */
    uint32_t brake_cycles;     /**< passes with a non-zero brake demand      */

    /* ---- APPEND ONLY BELOW (read_vehicle.cmake decodes by word index) ---- */
    uint32_t reset_reason;     /**< DRV_ResetReason - why this node last
                                *   booted. Read WITH uptime_ms: POWER_ON on
                                *   a node that has been running for minutes
                                *   is a brown-out, not a power-up.          */
    uint32_t uptime_ms;        /**< resets to 0 on every reset - the tell    */

    /* ---- loop latency, PEAK-HELD -------------------------------------
     * The main loop services CAN. If a pass takes longer than the command
     * timeout the node goes to FAILSAFE without anything on the bus being
     * wrong at all, so the longest pass of the run is a first-class safety
     * number, not a performance curiosity.
     *
     *   < 20 ms   healthy
     *   > 60 ms   long enough to have caused a FAILSAFE on its own
     *   > 300 ms  long enough for the gateway to declare this node absent
     *             (the station shows NO CAR)
     *
     * slow_passes counts overruns whatever caused them - the IMU is the
     * usual suspect but not the only one, so the counter does not name it. */
    uint32_t max_loop_ms;
    uint32_t slow_passes;

    /* ---- is this node REBOOTING, or losing the bus? ---------------------
     * The two look identical from the station and need opposite fixes, and
     * nothing on this part distinguishes them: the F103 has no brown-out flag,
     * so a supply sag reports the same POWER_ON as plugging the battery in.
     *
     * boot_count lives in .noinit, so it survives a warm reset and is garbage
     * on a cold one (hence the magic guard). Read it like this:
     *
     *   boot_count == 1, uptime climbing   the node has run since power-on.
     *                                      The dropouts are NOT resets - look
     *                                      at the CAN counters below.
     *   boot_count climbing during a drive THE NODE IS RESETTING. Supply, not
     *                                      software. Nothing below matters
     *                                      until that is fixed.
     */
    uint32_t boot_count;

    /* ---- CAN health, PEAK-HELD ------------------------------------------
     * bus_off_events counts ENTRIES into bus-off, not time spent there. ABOM
     * is enabled so the peripheral climbs out by itself once the bus goes
     * idle, which means a bus-off can come and go between two reads and leave
     * IsBusOff() saying false with nothing to show for it.
     *
     * While bus-off this node cannot transmit AT ALL, so the gateway stops
     * hearing its heartbeat and the station shows NO CAR. TEC reaching 255 is
     * how it gets there - a max_tec well above 0 means the bus is erroring
     * even when it has not yet fallen over. */
    uint32_t bus_off_events;
    uint32_t max_tec;
    uint32_t max_rec;

    /* Longest gap between two ACCEPTED 0x200 frames. Over 60 ms is a FAILSAFE
     * on its own. This one points AWAY from this node: the command comes from
     * the gateway, so a long gap here with a healthy loop and a quiet bus
     * means the GATEWAY stopped sending, not that this node stopped
     * listening. */
    uint32_t max_cmd_gap_ms;
    uint32_t failsafe_events;
} vc_debug_t;

extern volatile vc_debug_t g_vc_debug;

#endif /* VC_DEBUG_H_ */
