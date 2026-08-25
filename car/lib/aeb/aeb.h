/**
 ******************************************************************************
 * @file    aeb.h
 * @brief   Autonomous Emergency Braking decision logic.
 *
 * Pure C, no CMSIS, no hardware. Everything here is a function of numbers the
 * caller supplies, so it compiles and runs on a host and can be exercised
 * against a recorded bus trace without a car attached. The node owns the
 * actuators; this owns the decision.
 *
 * ## What triggers it
 *
 * Not a fixed time-to-collision. AEB_Update() computes a **last point to
 * brake** for the speed the encoders are measuring right now:
 *
 *     d_lpb = v^2 / (2a)   stopping distance at deceleration a
 *           + v * t_lat    ground covered before the brake takes effect
 *           + margin       standoff we intend to keep
 *
 * and compares the measured range against it. The difference from a TTC
 * threshold matters: TTC treats 300 mm/s and 3000 mm/s alike, but the distance
 * needed to stop from them differs by a factor of a hundred. LPB is the figure
 * the vehicle physically needs, which is why it is also published as
 * VCM_LpbDistance - the ten-run validation protocol can then be scored from a
 * bus trace alone.
 *
 * ## The cascade
 *
 * Staged, so the driver is warned before the car acts:
 *
 *     WARN     range <= d_lpb * warn_pct      no braking, tell the operator
 *     PARTIAL  range <= d_lpb * partial_pct   part brake, still recoverable
 *     FULL     range <= d_lpb                 full brake - the last moment
 *     LATCHED  <- entered from FULL           brake held, throttle locked out
 *
 * ## Why LATCHED exists
 *
 * Without it the system defeats itself. AEB brakes, the car stops, speed falls
 * below the minimum - and the trigger condition evaporates because the car is
 * no longer moving. State returns to IDLE and the driver's throttle, which has
 * been held down the whole time, drives straight into the obstacle that was
 * just avoided.
 *
 * LATCHED holds the intervention for a fixed cooldown after full braking
 * begins, regardless of what the speed does. Throttle is inhibited for that
 * whole window, so a held stick cannot re-launch the car the instant it stops.
 *
 * ## What the encoders are for
 *
 * Two jobs, both essential:
 *
 *   1. **Speed sets the threshold.** d_lpb is recomputed every cycle from the
 *      measured speed, so the intervention distance falls as the car slows -
 *      the system stops demanding brake once the remaining gap is genuinely
 *      enough.
 *   2. **Speed says when braking is done.** Once the wheels are below the
 *      stopped threshold the brake demand is released, because a shorted
 *      bridge against a stationary motor is heat and nothing else. The latch
 *      stays on; only the braking stops.
 *
 * Known limit, recorded on VCM_Speed in the DBC: encoders measure WHEEL
 * rotation. In a locked-wheel slide they read near zero while the car is still
 * moving. So "stopped" here means "the wheels have stopped", and d_lpb
 * collapses toward `margin` during a slide. This is the single biggest reason
 * `margin` is not zero.
 ******************************************************************************
 */
#ifndef AEB_H_
#define AEB_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Cascade stage. Values match VC_AebState in contracts/adas.dbc -
 *        do not renumber without changing the DBC.
 */
typedef enum {
    AEB_STATE_IDLE    = 0,
    AEB_STATE_WARN    = 1,
    AEB_STATE_PARTIAL = 2,
    AEB_STATE_FULL    = 3,
    AEB_STATE_LATCHED = 4
} AEB_State;

/**
 * @brief Tuning. The node fills this from vehicle_config.h.
 *
 * All integer: the F103 has no FPU, and this runs in the control loop.
 */
typedef struct {
    /* --- last-point-to-brake terms --- */
    uint32_t decel_mmps2;      /**< braking deceleration the car achieves     */
    uint32_t latency_ms;       /**< sense -> actuate, end to end              */
    uint16_t margin_mm;        /**< standoff to keep when stopped             */

    /* --- cascade thresholds, percent of d_lpb --- */
    uint16_t warn_pct;         /**< e.g. 200 = warn at twice d_lpb            */
    uint16_t partial_pct;      /**< e.g. 140                                  */
    uint16_t release_pct;      /**< hysteresis on de-escalation, e.g. 115     */

    /* --- gates --- */
    uint16_t min_speed_mmps;   /**< below this the AEB does not act           */
    uint16_t stopped_mmps;     /**< at/below this the wheels have stopped     */
    uint8_t  min_track_age;    /**< cycles a track must survive before acting */
    int16_t  opening_mmps;     /**< reject if receding faster than this       */
    uint16_t max_yaw_mrads;    /**< suppress above this turn rate; 0 disables */
    uint8_t  max_steer_pct;    /**< suppress above this steering command;
                                *   0 disables                               */

    /* --- intervention --- */
    uint8_t  partial_brake_pct;/**< brake demand in the PARTIAL stage         */
    uint32_t cooldown_ms;      /**< how long LATCHED holds after FULL         */
} AEB_Config;

/** @brief Everything the decision needs for one cycle. */
typedef struct {
    uint32_t now_ms;
    int16_t  speed_mmps;       /**< from the encoders. Positive is forward.   */
    int16_t  yaw_mrads;        /**< VCM_YawRate - MEASURED rotation.          */
    int8_t   steer_pct;        /**< driver's steering command, -100..+100.
                                *   COMMANDED - leads the measured yaw.       */
    uint16_t range_mm;         /**< SF_Range from the fused object            */
    int16_t  range_rate_mmps;  /**< SF_RangeRate. NEGATIVE means closing.     */
    uint8_t  track_age;        /**< SF_TrackAge                               */
    bool     object_valid;     /**< SF_Status == VALID, and the frame is fresh*/
} AEB_Input;

/**
 * @brief Why the AEB is not acting.
 *
 * Exists because "the AEB did not fire" is the one symptom that looks
 * identical for half a dozen unrelated causes, and guessing between them from
 * the outside costs a test run each time. Reported every cycle.
 */
typedef enum {
    AEB_BLOCK_NONE = 0,        /**< armed - not blocked                       */
    AEB_BLOCK_NO_OBJECT,       /**< SF_Status not VALID, or the frame is stale*/
    AEB_BLOCK_TRACK_AGE,       /**< target seen, but not for long enough      */
    AEB_BLOCK_TOO_SLOW,        /**< below min_speed, or reversing             */
    AEB_BLOCK_OPENING,         /**< target receding faster than noise         */
    AEB_BLOCK_YAW,             /**< MEASURED rotation too high               */
    AEB_BLOCK_STEER            /**< driver COMMANDED a turn - the array is
                                *   no longer looking where the car is going */
} AEB_Block;

/** @brief What the node should do about it. */
typedef struct {
    AEB_State state;
    uint8_t   brake_pct;       /**< 0..100 brake demand                       */
    bool      inhibit_throttle;/**< driver throttle must be forced to zero    */
    uint16_t  lpb_mm;          /**< d_lpb for this cycle -> VCM_LpbDistance   */
    AEB_Block block;           /**< why it is not acting, if it is not        */
} AEB_Output;

/** @brief Populate @p cfg with the built-in defaults. */
void AEB_ConfigDefaults(AEB_Config *cfg);

/**
 * @brief Reset to IDLE. Call once at start-up, and after any fault that makes
 *        the inputs untrustworthy.
 * @param cfg copied by pointer - must outlive the AEB.
 */
void AEB_Init(const AEB_Config *cfg);

/**
 * @brief Compute the last-point-to-brake distance for a speed.
 *
 * Exposed separately because it is published every cycle whether or not the
 * AEB is acting, and because it is the single most useful number for tuning:
 * drive at a known speed, read VCM_LpbDistance, compare against where the car
 * actually stops.
 *
 * @param cfg          tuning
 * @param speed_mmps   forward speed; negative or zero yields just the margin
 * @return distance in mm, saturated to the DBC's 4000 mm field range
 */
uint16_t AEB_LpbDistance(const AEB_Config *cfg, int16_t speed_mmps);

/**
 * @brief Advance the state machine one cycle.
 *
 * Call every control-loop pass, not only when a new object arrives - the latch
 * timer and the de-escalation both need to run on a stale object as well as a
 * fresh one.
 *
 * @param in  this cycle's measurements
 * @param out written on every call
 */
void AEB_Update(const AEB_Input *in, AEB_Output *out);

/** @brief Current stage, for callers that only need the state. */
AEB_State AEB_GetState(void);

/**
 * @brief True once the wheels are at rest during an intervention.
 *
 * Distinct from `state == LATCHED`: the latch runs for a fixed time, but this
 * says whether the car has actually come to rest inside it.
 */
bool AEB_IsStopped(void);

#ifdef __cplusplus
}
#endif

#endif /* AEB_H_ */
