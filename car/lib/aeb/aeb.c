/**
 ******************************************************************************
 * @file    aeb.c
 * @brief   AEB cascade and last-point-to-brake computation. See aeb.h.
 *
 * Integer arithmetic throughout. The F103 has no FPU, so every float here
 * would be a library call inside the control loop - the same reason the
 * encoder RPM conversion was moved out of its ISR.
 ******************************************************************************
 */
#include "aeb.h"

/* SF_Range and VCM_LpbDistance are both [0|4000] mm in adas.dbc. Saturate
 * rather than wrap: a d_lpb that rolls over reads as "brake later", which is
 * the one direction this number must never lie in. */
#define LPB_MAX_MM   4000u

static AEB_Config s_cfg;
static AEB_State  s_state;
static uint32_t   s_latch_start_ms;
static uint8_t    s_latch_brake_pct;   /* strongest demand of this event */
static bool       s_stopped;

/* -------------------------------------------------------------------------- */
/*  Library defaults                                                          */
/* -------------------------------------------------------------------------- */
/*
 * A SAFE FALLBACK, NOT ANY PARTICULAR VEHICLE'S TUNING. These exist so the
 * host tests and a node that has not configured anything still get sensible
 * behaviour instead of a zeroed struct - where a zero decel or margin would
 * silently collapse d_lpb.
 *
 * The car's real numbers live in car/nodes/vehicle_control/aeb_tuning.h and
 * override every one of these via CAR_AEB_LoadConfig(). These values
 * DELIBERATELY do not track that file: they are the library's floor, and
 * chasing the vehicle's tuning here would just recreate the two-sources-of-
 * truth problem that header was written to remove.
 */
#define AEB_DEFAULT_DECEL_MMPS2     3000u  /* ~0.3 g, conservative            */
#define AEB_DEFAULT_LATENCY_MS      120u   /* sense -> actuate, rounded UP    */
#define AEB_DEFAULT_MARGIN_MM       150u   /* standoff, and the d_lpb floor   */

#define AEB_DEFAULT_WARN_PCT        200u   /* warn at 2.0x d_lpb              */
#define AEB_DEFAULT_PARTIAL_PCT     140u   /* part brake at 1.4x              */
#define AEB_DEFAULT_RELEASE_PCT     115u   /* de-escalation hysteresis        */

#define AEB_DEFAULT_MIN_SPEED_MMPS  150u
#define AEB_DEFAULT_STOPPED_MMPS    60u
#define AEB_DEFAULT_MIN_TRACK_AGE   3u
/* Reject only a target receding FASTER than range-rate noise. Deliberately not
 * a "must be closing" gate: SF_RangeRate is a plain first difference, and one
 * noisy positive sample must not disarm the AEB when it is needed. */
#define AEB_DEFAULT_OPENING_MMPS    200
#define AEB_DEFAULT_MAX_YAW_MRADS   1500u  /* ~86 deg/s - a deliberate turn   */
#define AEB_DEFAULT_MAX_STEER_PCT   40u    /* percent of full stick           */

#define AEB_DEFAULT_PARTIAL_BRAKE   50u
#define AEB_DEFAULT_COOLDOWN_MS     3000u

void AEB_ConfigDefaults(AEB_Config *cfg)
{
    if (cfg == 0) {
        return;
    }

    cfg->decel_mmps2       = AEB_DEFAULT_DECEL_MMPS2;
    cfg->latency_ms        = AEB_DEFAULT_LATENCY_MS;
    cfg->margin_mm         = AEB_DEFAULT_MARGIN_MM;

    cfg->warn_pct          = AEB_DEFAULT_WARN_PCT;
    cfg->partial_pct       = AEB_DEFAULT_PARTIAL_PCT;
    cfg->release_pct       = AEB_DEFAULT_RELEASE_PCT;

    cfg->min_speed_mmps    = AEB_DEFAULT_MIN_SPEED_MMPS;
    cfg->stopped_mmps      = AEB_DEFAULT_STOPPED_MMPS;
    cfg->min_track_age     = AEB_DEFAULT_MIN_TRACK_AGE;
    cfg->opening_mmps      = AEB_DEFAULT_OPENING_MMPS;
    cfg->max_yaw_mrads     = AEB_DEFAULT_MAX_YAW_MRADS;
    cfg->max_steer_pct     = AEB_DEFAULT_MAX_STEER_PCT;

    cfg->partial_brake_pct = AEB_DEFAULT_PARTIAL_BRAKE;
    cfg->cooldown_ms       = AEB_DEFAULT_COOLDOWN_MS;
}

void AEB_Init(const AEB_Config *cfg)
{
    if (cfg != 0) {
        s_cfg = *cfg;
    } else {
        AEB_ConfigDefaults(&s_cfg);
    }

    s_state           = AEB_STATE_IDLE;
    s_latch_start_ms  = 0u;
    s_latch_brake_pct = 0u;
    s_stopped         = false;
}

uint16_t AEB_LpbDistance(const AEB_Config *cfg, int16_t speed_mmps)
{
    uint32_t v;
    uint32_t d;

    if ((cfg == 0) || (speed_mmps <= 0)) {
        return (cfg == 0) ? 0u : cfg->margin_mm;
    }

    v = (uint32_t)speed_mmps;

    /* v^2 / 2a. At the DBC's 3000 mm/s ceiling v*v is 9e6, comfortably inside
     * uint32 - but the divisor is caller-supplied, so guard it. */
    d = cfg->margin_mm;
    if (cfg->decel_mmps2 > 0u) {
        d += (v * v) / (2u * cfg->decel_mmps2);
    }

    /* Ground covered during the reaction delay. */
    d += (v * cfg->latency_ms) / 1000u;

    return (d > LPB_MAX_MM) ? (uint16_t)LPB_MAX_MM : (uint16_t)d;
}

/** @brief scale a distance by a percentage, saturating. */
static uint32_t pct_of(uint32_t value, uint16_t pct)
{
    const uint32_t r = (value * (uint32_t)pct) / 100u;
    return (r > LPB_MAX_MM) ? LPB_MAX_MM : r;
}

/**
 * @brief Are the preconditions for acting met at all?
 *
 * Every one of these is a false-positive guard, and the validation protocol
 * counts false positives, so they are as load-bearing as the trigger itself.
 * Returns the FIRST reason it is blocked, or AEB_BLOCK_NONE.
 */
static AEB_Block aeb_block_reason(const AEB_Input *in)
{
    if (!in->object_valid) {
        return AEB_BLOCK_NO_OBJECT;    /* nothing believable in view */
    }
    if (in->track_age < s_cfg.min_track_age) {
        return AEB_BLOCK_TRACK_AGE;    /* one-frame ghost, not a target yet */
    }
    if (in->speed_mmps < (int16_t)s_cfg.min_speed_mmps) {
        return AEB_BLOCK_TOO_SLOW;     /* stationary or reversing: the front
                                        * array is not looking where we are
                                        * going, so it must not brake */
    }
    if (in->range_rate_mmps > s_cfg.opening_mmps) {
        return AEB_BLOCK_OPENING;      /* clearly opening */
    }

    /* ---- turning: the array stops answering the right question ------------
     *
     * Every sensor on this node faces FORWARD, so the array looks where the
     * car is POINTING. That is only where the car is GOING while it is
     * driving straight. In a turn the two diverge, and the array sweeps
     * across scenery the car will never reach.
     *
     * That is not a noise problem to be filtered - it is a geometry problem.
     * The fused object is a plain minimum over four elements with no bearing
     * information, so there is nothing in the signal that could distinguish
     * "obstacle in my path" from "wall I am turning alongside". Braking on the
     * second is a false positive, and no amount of tuning downstream fixes it.
     *
     * TWO gates, because each covers the other's blind spot:
     *
     *   steer_pct  what the driver ASKED for. Available the instant the stick
     *              moves, needs no IMU, and leads the rotation it will cause.
     *   yaw_mrads  what the car is ACTUALLY doing. Catches rotation nobody
     *              commanded - a wheel losing grip, a kerb strike, a pivot -
     *              and works when the steering command is stale.
     *
     * The trade is explicit and not free: a genuine obstacle met mid-turn is
     * ignored too. Both thresholds are therefore set to catch a DELIBERATE
     * turn, not the small corrections a straight run is made of - a gate that
     * trips on trim would silently disable the AEB for most of a test run.
     * Either can be disabled with 0.
     *
     * Both are stand-ins for bearing-aware fusion in car/lib/tracker, which
     * would let an in-path test replace them. */
    if (s_cfg.max_steer_pct > 0u) {
        const int32_t steer = (in->steer_pct < 0)
                                ? -(int32_t)in->steer_pct
                                :  (int32_t)in->steer_pct;
        if (steer > (int32_t)s_cfg.max_steer_pct) {
            return AEB_BLOCK_STEER;
        }
    }

    if (s_cfg.max_yaw_mrads > 0u) {
        const int32_t yaw = (in->yaw_mrads < 0)
                              ? -(int32_t)in->yaw_mrads
                              :  (int32_t)in->yaw_mrads;
        if (yaw > (int32_t)s_cfg.max_yaw_mrads) {
            return AEB_BLOCK_YAW;
        }
    }

    return AEB_BLOCK_NONE;
}

void AEB_Update(const AEB_Input *in, AEB_Output *out)
{
    uint16_t lpb;
    uint32_t warn_mm;
    uint32_t partial_mm;
    uint32_t range;
    bool     armed;

    if ((in == 0) || (out == 0)) {
        return;
    }

    lpb   = AEB_LpbDistance(&s_cfg, in->speed_mmps);
    range = (uint32_t)in->range_mm;
    out->block = aeb_block_reason(in);
    armed = (out->block == AEB_BLOCK_NONE);

    /* Wheels at rest. Latched by the caller's own speed signal rather than by
     * elapsed time, because how long a stop takes depends on the speed it
     * started from. */
    s_stopped = ((in->speed_mmps <= (int16_t)s_cfg.stopped_mmps) &&
                 (in->speed_mmps >= -(int16_t)s_cfg.stopped_mmps));

    /* ---- LATCHED runs to its own clock and ignores everything else --------
     * Deliberately evaluated BEFORE the cascade: the whole point of the latch
     * is that it survives the trigger condition disappearing, which it always
     * does the moment the car stops. */
    if (s_state == AEB_STATE_LATCHED) {
        const uint32_t held = in->now_ms - s_latch_start_ms;

        /* Escalation still works inside the latch: if the gap keeps closing to
         * the last point to brake, a partial intervention becomes a full one.
         * The latch floors the demand; it does not cap it. */
        if (armed && (range <= (uint32_t)lpb)) {
            s_latch_brake_pct = 100u;
        }

        if (held < s_cfg.cooldown_ms) {
            out->state = AEB_STATE_LATCHED;
            /* Brake until the wheels stop, then release. Holding a shorted
             * bridge against a stationary motor produces heat and no useful
             * force. The THROTTLE stays locked either way - that is what the
             * cooldown is for. */
            out->brake_pct        = s_stopped ? 0u : s_latch_brake_pct;
            out->inhibit_throttle = true;
            out->lpb_mm           = lpb;
            return;
        }

        /* Cooldown served. Drop to IDLE and let this same cycle re-evaluate
         * from scratch - if the obstacle is still there and the car is still
         * moving, the cascade below will pick it straight back up. */
        s_state           = AEB_STATE_IDLE;
        s_latch_brake_pct = 0u;
    }

    if (!armed) {
        /* Disarmed mid-cascade - target lost, or the car has slowed below the
         * minimum. Fall back to IDLE. Note this cannot strand an intervention:
         * anything that reached FULL is in LATCHED, handled above. */
        s_state               = AEB_STATE_IDLE;
        out->state            = AEB_STATE_IDLE;
        out->brake_pct        = 0u;
        out->inhibit_throttle = false;
        out->lpb_mm           = lpb;
        return;
    }

    warn_mm    = pct_of(lpb, s_cfg.warn_pct);
    partial_mm = pct_of(lpb, s_cfg.partial_pct);

    /* ---- escalation ------------------------------------------------------
     * Checked closest-first so the most severe stage that applies wins. */
    /* ---- the braking stages LATCH; the warning stage does not -------------
     * Both PARTIAL and FULL start the cooldown, for the same reason the latch
     * exists at all - it is just far easier to miss for PARTIAL.
     *
     * d_lpb is a function of the CURRENT speed, so the instant braking takes
     * effect the threshold starts retreating: brake from 800 mm/s and d_lpb
     * falls 352 -> 180 mm while the car covers barely any ground. The range
     * that triggered the intervention is then outside the threshold that
     * triggered it, the cascade de-escalates, the brake releases, and the
     * driver's throttle comes back with the obstacle still sitting there.
     *
     * A partial brake that cancels itself the moment it starts working is
     * worse than no partial brake at all. Latching on entry is what lets the
     * intervention outlive its own effect. */
    if (range <= (uint32_t)lpb) {
        s_state           = AEB_STATE_LATCHED;
        s_latch_start_ms  = in->now_ms;
        s_latch_brake_pct = 100u;

        out->state            = AEB_STATE_FULL;   /* report the CAUSE this
                                                   * cycle; LATCHED from the
                                                   * next one on */
        out->brake_pct        = 100u;
        out->inhibit_throttle = true;
        out->lpb_mm           = lpb;
        return;
    }

    if (range <= partial_mm) {
        s_state           = AEB_STATE_LATCHED;
        s_latch_start_ms  = in->now_ms;
        s_latch_brake_pct = s_cfg.partial_brake_pct;

        out->state            = AEB_STATE_PARTIAL;
        out->brake_pct        = s_cfg.partial_brake_pct;
        out->inhibit_throttle = true;
        out->lpb_mm           = lpb;
        return;
    }

    /* Hysteresis on the one boundary that can still chatter. The braking
     * stages latch, so they cannot flap; WARN <-> IDLE can, and it drives the
     * operator's warning indicator. */
    if (range <= warn_mm) {
        s_state = AEB_STATE_WARN;
    } else {
        s_state = (range <= pct_of(warn_mm, s_cfg.release_pct))
                    ? AEB_STATE_WARN
                    : AEB_STATE_IDLE;
    }

    /* WARN is a notification, not an intervention: no brake, and the driver
     * keeps the throttle. */
    out->state            = s_state;
    out->brake_pct        = 0u;
    out->inhibit_throttle = false;
    out->lpb_mm           = lpb;
}

AEB_State AEB_GetState(void)
{
    return s_state;
}

bool AEB_IsStopped(void)
{
    return s_stopped;
}
