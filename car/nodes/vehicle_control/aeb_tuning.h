/**
 ******************************************************************************
 * @file    aeb_tuning.h
 * @brief   AEB tuning for the vehicle-control node - the ONLY place these
 *          numbers live.
 *
 * Split out of vehicle_config.h because these are not board wiring. Every one
 * of them changes when and how hard the car brakes by itself, and they are
 * read together when tuning; having them share a file with pin assignments
 * meant scrolling past PWM channels to find a safety parameter.
 *
 * ## Why the loader at the bottom exists
 *
 * AEB_Config has fourteen fields, and before this file they were written out
 * in THREE places: the library defaults in AEB_ConfigDefaults(), the macros
 * here, and a fourteen-line copy block in main.c. Adding a field meant
 * remembering all three. Miss the copy block and the new field silently keeps
 * the library default instead of the node's value - a divergence that
 * compiles, links, runs, and is invisible until the car does not brake the way
 * the config says it should.
 *
 * CAR_AEB_LoadConfig() puts the values and the assignment that consumes them
 * in one place, so a new parameter is one edit rather than three that must
 * agree.
 *
 * The library keeps its own AEB_DEFAULT_* set for host tests and for any node
 * that does not override. Those are a fallback, NOT this vehicle's tuning -
 * they deliberately do not track the values below.
 ******************************************************************************
 */
#ifndef AEB_TUNING_H_
#define AEB_TUNING_H_

#include "adas.h"   /* cycle times come from the DBC, not from here */
#include "aeb.h"    /* AEB_Config, for the loader below             */

/* ===== AEB tuning ==========================================================
 * Consumed by car/lib/aeb. Every one of these is a safety parameter; the
 * defaults in AEB_ConfigDefaults() match, and this block overrides them.
 *
 * The trigger is a speed-dependent LAST POINT TO BRAKE, not a fixed TTC:
 *
 *     d_lpb = v^2 / (2 * DECEL) + v * LATENCY + MARGIN
 *
 * Published every cycle as VCM_LpbDistance so the ten-run protocol can be
 * scored from a bus trace. */

#define CAR_AEB_DECEL_MMPS2     5500u

/* Sense to actuate: VL53L0X 50 ms budget + fused publish + CAN + a loop pass. */
#define CAR_AEB_LATENCY_MS      120u

#define CAR_AEB_MARGIN_MM       120u

/* Cascade entry points, as a percentage of d_lpb. */
#define CAR_AEB_WARN_PCT        200u      /* warn at 2.0x   */
#define CAR_AEB_PARTIAL_PCT     140u      /* part brake 1.4x*/
#define CAR_AEB_RELEASE_PCT     115u      /* de-escalation hysteresis         */

#define CAR_AEB_MIN_SPEED_MMPS  60u
#define CAR_AEB_STOPPED_MMPS    60u
#define CAR_AEB_MIN_TRACK_AGE   3u
#define CAR_AEB_OPENING_MMPS    200       /* reject only if clearly receding  */

#define CAR_AEB_PARTIAL_BRAKE   50u       /* brake % in the PARTIAL stage     */

/* Suppress the AEB above this turn rate (~86 deg/s). 0 disables the gate.*/
#define CAR_AEB_MAX_YAW_MRADS   1500u

/* Suppress the AEB above this steering command, as a percentage of full stick.
 * 0 disables the gate. */
#define CAR_AEB_MAX_STEER_PCT   40u

/* How long the intervention is HELD after full braking begins */
#define CAR_AEB_COOLDOWN_MS     3000u

/* 0x100 is event-triggered (~13 ms), so this is a watchdog, not a schedule.
 * Past it the object is stale, the AEB is disarmed and FAULT_FRONT_TIMEOUT is
 * raised. Matches the gateway's SF_OBJECT_TIMEOUT_MS. */
#define CAR_SF_OBJECT_TIMEOUT_MS (5u * ADAS_SENSOR_FRONT_OBJECT_CYCLE_TIME_MS)

/* -------------------------------------------------------------------------- */
/*  Loader                                                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief Fill @p cfg from the macros above. Every field, in struct order.
 */
static inline void CAR_AEB_LoadConfig(AEB_Config *cfg)
{
    cfg->decel_mmps2       = CAR_AEB_DECEL_MMPS2;
    cfg->latency_ms        = CAR_AEB_LATENCY_MS;
    cfg->margin_mm         = CAR_AEB_MARGIN_MM;

    cfg->warn_pct          = CAR_AEB_WARN_PCT;
    cfg->partial_pct       = CAR_AEB_PARTIAL_PCT;
    cfg->release_pct       = CAR_AEB_RELEASE_PCT;

    cfg->min_speed_mmps    = CAR_AEB_MIN_SPEED_MMPS;
    cfg->stopped_mmps      = CAR_AEB_STOPPED_MMPS;
    cfg->min_track_age     = CAR_AEB_MIN_TRACK_AGE;
    cfg->opening_mmps      = CAR_AEB_OPENING_MMPS;
    cfg->max_yaw_mrads     = CAR_AEB_MAX_YAW_MRADS;
    cfg->max_steer_pct     = CAR_AEB_MAX_STEER_PCT;

    cfg->partial_brake_pct = CAR_AEB_PARTIAL_BRAKE;
    cfg->cooldown_ms       = CAR_AEB_COOLDOWN_MS;
}

/* The cascade must widen outwards, or a stage is unreachable: FULL fires at
 * 100% of d_lpb, so PARTIAL and WARN have to sit further out than that and
 * than each other. */
_Static_assert(CAR_AEB_WARN_PCT > CAR_AEB_PARTIAL_PCT,
               "WARN must trigger further out than PARTIAL");
_Static_assert(CAR_AEB_PARTIAL_PCT > 100u,
               "PARTIAL must trigger further out than FULL (100% of d_lpb)");

/* d_lpb divides by decel. Zero is guarded at runtime, but a zero here is a
 * configuration mistake worth refusing to build. */
_Static_assert(CAR_AEB_DECEL_MMPS2 > 0u, "decel of 0 removes the braking term");

/* A brake demand above 100% is a duty cycle that does not exist. */
_Static_assert(CAR_AEB_PARTIAL_BRAKE <= 100u, "brake demand is a percentage");

#endif /* AEB_TUNING_H_ */
