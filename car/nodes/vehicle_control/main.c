/**
 ******************************************************************************
 * @file    main.c
 * @brief   Vehicle-control node: the only node that touches the actuators.
 *
 * Runs on an STM32F103. Took the motors and encoders off the gateway when the
 * car was split across three MCUs.
 *
 *   1. brings up the clock, tick, status LED, CAN, motors, encoders and IMU,
 *   2. receives 0x200 Gateway_DriverCmd and validates its E2E protection,
 *   3. arbitrates - today the driver always wins, because the sensor node is
 *      not integrated yet and AEB has no input,
 *   4. mixes the winning intent onto the two wheels, and
 *   5. publishes what it actually did, plus measured motion.
 *
 * Publishes : 0x300 VC_Status (20 ms), 0x310 VC_Motion (20 ms),
 *             0x701 VC_Heartbeat (100 ms)
 * Consumes  : 0x200 Gateway_DriverCmd
 *
 * THREE independent things stop the car, and any one of them is sufficient:
 *   - CAN timeout : no valid 0x200 for CAR_CAN_CMD_TIMEOUT_MS
 *   - link flag   : the gateway reports GW_LinkOk = 0
 *   - disarmed    : GW_Armed clear
 *
 * Yaw rate comes from the MPU6050 when one is fitted, and falls back to the
 * encoder-derived kinematic estimate when it is not. The fallback is silent by
 * design: an absent IMU degrades the measurement, it does not stop the car.
 *
 * All pins live in vehicle_config.h.
 *
 * Status LED (PA5): 2 flashes = clock failed, 4 = CAN failed, brief flash =
 * no command, ~4 Hz = commands arriving but disarmed, solid = armed.
 ******************************************************************************
 */
#include "drv_common.h"
#include "drv_clock.h"
#include "drv_gpio.h"
#include "drv_pwm.h"
#include "drv_timer.h"
#include "drv_systick.h"
#include "drv_can.h"
#include "drv_i2c.h"

#include "vehicle_config.h"
#include "control_app.h"    /* differential-drive mixer (car/lib/mixer)      */
#include "motor_encoder.h"  /* wheel speed (car/devices/tb6612)              */
#include "mpu6050.h"        /* measured yaw rate (car/devices/mpu6050)       */
#include "tb6612_motor.h"   /* TB6612_BrakeLevel - the AEB actuator          */
#include "aeb.h"            /* AEB cascade (car/lib/aeb)                     */
#include "aeb_tuning.h"     /* this vehicle's AEB numbers + the loader       */
#include "vc_debug.h"       /* peak-hold SWD view - why did the AEB not fire? */

#include "adas.h"           /* generated from contracts/adas.dbc             */
#include "e2e.h"

#include <string.h>

#define forever for (;;)

/* Wheel travel per revolution. Held as a float because the encoder module
 * already pulls in soft-float for its RPM arithmetic. */
#define WHEEL_CIRCUM_MM   (3.14159265f * CAR_WHEEL_DIAMETER_MM)

/* ---- Received command ---------------------------------------------------- */
static struct adas_gateway_driver_cmd_t s_cmd;
static uint32_t s_last_cmd_ms;
static bool     s_have_cmd;
static E2E_Receiver s_rx_cmd;

/* ---- Node state ---------------------------------------------------------- */
static bool     s_clock_ok;
static bool     s_can_ok;
/* Captured ONCE at start-up, before anything can clear it. Published in the
 * heartbeat so a node that reset mid-run says so on the bus - the difference
 * between "the link dropped" and "the node rebooted" is invisible otherwise,
 * and they need completely different fixes. */
static DRV_ResetReason s_reset_reason;

static uint8_t  s_faults;            /* VC_Faults bitfield                   */
static uint8_t  s_e2e_errors;        /* saturating, published in 0x300       */
static uint32_t s_last_e2e_err_ms;   /* when FAULT_E2E was last (re)raised   */
static uint32_t s_tx_dropped;

/* ---- What we actually applied, for 0x300 --------------------------------- */
static uint8_t  s_applied_throttle;
static uint8_t  s_applied_brake;
static int8_t   s_applied_steering;

/* ---- Measured motion, for 0x310 ------------------------------------------ */
static int16_t  s_speed_mmps;
static int16_t  s_yaw_mrads;

/* ---- Front fused object (0x100) - the AEB input --------------------------
 * Tracked with its own arrival time because 0x100 is EVENT-triggered: the
 * sensor node publishes whenever an element reports, so absence is the only
 * signal that the object list has died, and it must disarm the AEB rather
 * than leave it acting on the last range it happened to see. */
static struct adas_sensor_front_object_t s_sf_object;
static uint32_t s_last_sf_obj_ms;
static bool     s_seen_sf_obj;
static E2E_Receiver s_rx_sf_obj;

/* ---- AEB ----------------------------------------------------------------- */
static AEB_Config s_aeb_cfg;
static AEB_Output s_aeb;            /* refreshed every control-loop pass     */

/* ---- SWD diagnostic ------------------------------------------------------
 * Peak-held, because the cycle that matters is the closest approach and it is
 * long gone by the time a debugger halts the car. See vc_debug.h. */
volatile vc_debug_t g_vc_debug;

/* .noinit: NOT zeroed by the startup code, so it survives a warm reset. On a
 * cold start the contents are undefined, which is what the magic guards. */
#define BOOT_MAGIC  0x424F4F54u   /* "BOOT" */
__attribute__((section(".noinit"))) static volatile uint32_t s_boot_magic;
__attribute__((section(".noinit"))) static volatile uint32_t s_boot_count;

static uint32_t s_loop_passes;
static uint32_t s_prev_pass_ms;
static uint32_t s_slow_passes;
static uint32_t s_can_rx_object;
static uint32_t s_brake_cycles;

/* ---- IMU ------------------------------------------------------------------
 * ONE job: a measured yaw rate. The kinematic estimate from the wheel-speed
 * difference reads zero for a skidding wheel, which is exactly the case an AEB
 * stop produces, so a gyro is worth having for that alone.
 *
 * It is deliberately NOT in the speed path. Fusing integrated acceleration into
 * VCM_Speed worked as designed and still read ~100 mm/s at a standstill: the
 * filter settles to bias x TAU, and a mounting tilt of barely a degree leaks
 * enough gravity into the forward axis to produce that. Speed is encoder-only,
 * and the AEB brakes on a number with no drift term in it.
 */
/* Consecutive failed reads before the IMU is dropped for this retry window.
 *
 * WAS 10, and that was a latency bug rather than a tuning choice. Each failed
 * read blocks the main loop for the I2C timeout budget, so ten in a row is ten
 * times that - and the loop it blocks is the one servicing CAN. Ten misses was
 * enough on its own to miss the 60 ms command timeout and the gateway's 300 ms
 * heartbeat window, which is how motor noise on the IMU leads ended up
 * stopping the car and painting NO CAR on the station.
 *
 * Three still rides out an isolated NAK, which is what the tolerance is for,
 * while bounding the worst case to well under the command timeout. An optional
 * diagnostic sensor gets a few chances, not an unlimited claim on the loop. */
#define IMU_MISS_LIMIT   3u
#define IMU_PERIOD_MS    10u   /* 100 Hz, matching CAR_ENC_SAMPLE_HZ        */
#define IMU_RETRY_MS     2000u /* how often to try to get it back            */

/* Read by motion_update() whichever way CAR_IMU_ENABLED is set: with the IMU
 * compiled out s_imu_ok is simply never true and the kinematic estimate wins
 * the ternary, so the fallback needs no second code path. */
static bool     s_imu_ok;
static float    s_yaw_imu_mrads;    /* last good measured yaw rate          */

#if CAR_IMU_ENABLED
static float    s_gyro_z_bias;      /* zero-rate offset, raw counts         */
static uint8_t  s_imu_misses;
static uint32_t s_next_imu_ms;
static uint32_t s_next_imu_retry_ms;
static uint8_t  s_imu_who;          /* whatever WHO_AM_I actually returned  */

/* Bias capture runs INCREMENTALLY in imu_poll(), not as a blocking loop in
 * imu_init(). Two reasons, both learned the hard way:
 *
 *   - it was all-or-nothing. One failed read out of 64 aborted the whole
 *     bring-up permanently, and nothing ever retried. On a bench with 5 cm of
 *     wire that never happens; in the car, on longer unshielded leads beside
 *     motor wiring, one NAK is enough to lose the IMU for the whole run.
 *   - it blocked for ~130 ms inside board_init() for no good reason.
 *
 * Dropped samples now simply do not contribute. The average just takes
 * slightly longer to fill. */
static int32_t  s_bias_sum_gz;
static uint16_t s_bias_n;
static bool     s_bias_done;
#endif /* CAR_IMU_ENABLED */

/* ---- CAN transmit scheduling --------------------------------------------- */
static uint8_t  s_alv_status;
static uint8_t  s_alv_motion;
static uint8_t  s_alv_hb;
static uint32_t s_next_status_ms;
static uint32_t s_next_hb_ms;

/* VC_Faults bit positions - keep in step with the DBC comment on VC_Faults. */
#define FAULT_BUS_OFF       (1u << 0)
#define FAULT_FRONT_TIMEOUT (1u << 1)
/* bit 2 reserved: side sensor, no side node in the three-MCU build */
#define FAULT_GW_TIMEOUT    (1u << 3)
#define FAULT_E2E           (1u << 4)
#define FAULT_CLOCK         (1u << 7)

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

static bool blink_code(uint32_t ms, uint32_t count)
{
    const uint32_t on = 150u, off = 150u, gap = 900u;
    const uint32_t burst = count * (on + off);
    const uint32_t t = ms % (burst + gap);
    return (t < burst) && ((t % (on + off)) < on);
}

/** @brief True while driver intent is arriving on time AND the radio is up. */
static bool command_ok(void)
{
    return s_have_cmd
        && !DRV_SysTick_Elapsed(s_last_cmd_ms, CAR_CAN_CMD_TIMEOUT_MS)
        && (s_cmd.gw_link_ok != 0u);
}

static void status_led_update(void)
{
    const uint32_t ms = DRV_SysTick_GetTick();
    bool on;

    if (!s_clock_ok) {
        on = blink_code(ms, 2u);
    } else if (!s_can_ok) {
        on = blink_code(ms, 4u);
    } else if (!command_ok()) {
        on = (ms % 1000u) < 60u;
    } else if (s_cmd.gw_armed != 0u) {
        on = true;
    } else {
        on = (ms & 0x80u) != 0u;
    }

    led_status_write(on);
}

/* -------------------------------------------------------------------------- */
/*  Actuation                                                                 */
/* -------------------------------------------------------------------------- */

/** @brief Everything off. Link loss, disarmed, or any fault path. */
static void outputs_stop(void)
{
    ControlApp_Stop();                     /* coast both wheels */
    DRV_PWM_SetDuty(CAR_DRIVE_TIMER, CAR_DRIVE_CHANNEL, 0u);
    DRV_GPIO_Write(CAR_REVERSE_LED_PORT, CAR_REVERSE_LED_PIN, GPIO_LOW);

    s_applied_throttle = 0u;
    s_applied_brake    = 0u;
    s_applied_steering = 0;
}

/**
 * @brief True while the AEB is overriding the driver on the longitudinal axis.
 *
 * WARN is deliberately excluded: it is a notification, not an intervention,
 * and the driver still has the car. Only the stages that actually take
 * authority count as winning the axis.
 */
static bool aeb_has_long_axis(void)
{
    return (s_aeb.brake_pct > 0u) || s_aeb.inhibit_throttle;
}

/**
 * @brief Shape the operator's steering into what the mixer is given.
 *
 * Two stages, addressing two different complaints. GAIN narrows the authority
 * so full stick is a turn rather than a pivot; SLEW limits how fast that value
 * may move, which is what removes the snap from a flicked stick.
 *
 * The slew step is derived from the ACTUAL elapsed time rather than an assumed
 * loop rate. The main loop is not periodic - it is as fast as CAN, the IMU and
 * the encoders let it be - so a fixed per-pass step would make the rate depend
 * on how busy the node happens to be, and change the car's handling whenever
 * something unrelated was added to the loop.
 */
static int8_t steering_shape(int8_t raw, uint32_t now_ms)
{
    static int32_t s_applied;        /* the shaped value, held between passes */
    static uint32_t s_last_ms;
    static bool s_primed;

    int32_t target = ((int32_t)raw * (int32_t)CAR_STEER_GAIN_PCT) / 100;
    uint32_t dt_ms;
    int32_t step;

    if (!s_primed) {
        s_primed  = true;
        s_last_ms = now_ms;
        s_applied = target;
        return (int8_t)target;
    }

    dt_ms     = now_ms - s_last_ms;
    s_last_ms = now_ms;

    if (CAR_STEER_SLEW_PCT_PER_S == 0u) {
        s_applied = target;
    } else {
        step = (int32_t)((CAR_STEER_SLEW_PCT_PER_S * dt_ms) / 1000u);
        if (step < 1) {
            step = 1;                /* never stall: sub-millisecond passes
                                      * would otherwise freeze the steering */
        }
        if (target > (s_applied + step)) {
            s_applied += step;
        } else if (target < (s_applied - step)) {
            s_applied -= step;
        } else {
            s_applied = target;
        }
    }

    return (int8_t)s_applied;
}

/**
 * @brief Keep the shaper tracking while the wheels are not being steered.
 *
 * Called on the stopped and intervening paths so the held value decays to
 * centre instead of going stale. Without it the shaper would still hold full
 * lock from before a link loss, and hand that straight back to the mixer the
 * moment control resumed.
 */
static void steering_relax(uint32_t now_ms)
{
    (void)steering_shape(0, now_ms);
}

/**
 * @brief Apply the arbitrated command to the motors.
 *
 * Two bidders on the longitudinal axis: the driver, and the AEB. The
 * arbitration is not a blend - the AEB simply outranks the driver whenever it
 * is intervening, and the winner is published as VC_ArbWinnerLong.
 *
 * AN INTERVENTION TAKES THE YAW AXIS TOO, which is forced by the platform
 * rather than chosen. On a differential drive, steering IS a wheel-speed
 * difference, so:
 *
 *   - while braking, a symmetric short across both motors removes exactly the
 *     difference that steering is made of. There is no short that brakes and
 *     turns at once;
 *   - once throttle is inhibited, a steering command is no longer a turn at
 *     all. ControlApp_Drive(0, steering) gives the wheels equal and opposite
 *     speeds - the car spins on the spot. That is not steering around an
 *     obstacle, it is a pirouette in front of one.
 *
 * So the driver's steering is dropped for the whole intervention and the
 * wheels are coasted instead. On an Ackermann vehicle this trade would not
 * exist and yaw could stay with the driver throughout - one of the places the
 * plant model genuinely does not transfer.
 */
static void outputs_apply(uint32_t now_ms)
{
    /* Safety gate: a disarmed car ignores drive commands entirely. Note this
     * also clears any AEB braking - standby drops the bridges, and a disarmed
     * car has no business holding a short across its motors. */
    if (!command_ok() || (s_cmd.gw_armed == 0u)) {
        outputs_stop();
        steering_relax(now_ms);
        return;
    }

    /* The frame carries throttle as an unsigned magnitude plus a reverse bit;
     * the mixer wants a signed throttle. Brake wins over throttle - pressing
     * both stops rather than drives. */
    const bool reverse = (s_cmd.gw_reverse != 0u);
    uint8_t    thr     = (uint8_t)s_cmd.gw_throttle;

    if (s_cmd.gw_brake > 0u) {
        thr = 0u;
    }

    /* ---- AEB arbitration ------------------------------------------------
     * Throttle is forced to zero BEFORE anything reaches the mixer. Cutting
     * drive and applying the brake in the same pass matters: a shorted bridge
     * fighting a powered one is heat in the TB6612 and no braking.
     *
     * REVERSE IS THE ONE EXCEPTION, and it has to be. The front array does not
     * look backwards, so nothing the AEB knows about is behind the car -
     * backing away is the correct escape from the obstacle it just stopped
     * for. Without this the cooldown traps the car against the wall for three
     * seconds with the operator unable to do anything about it, which reads
     * from the driving seat as "the AEB fires when I reverse".
     *
     * Only once the wheels are at rest, though: while the car is still rolling
     * forwards a reverse command would be fighting the brake, not escaping. */
    const bool reverse_escape = reverse && AEB_IsStopped();

    if (s_aeb.inhibit_throttle && !reverse_escape) {
        thr = 0u;
    }

    const int8_t throttle = reverse ? -(int8_t)thr : (int8_t)thr;
    const int8_t steer    = steering_shape((int8_t)s_cmd.gw_steering, now_ms);

    if (reverse_escape) {
        /* Backing away under the operator's control. The latch still holds -
         * the cooldown timer is untouched and forward throttle stays locked -
         * but the car is no longer pinned. */
        ControlApp_Drive(throttle, steer);
        DRV_PWM_SetDuty(CAR_DRIVE_TIMER, CAR_DRIVE_CHANNEL, thr);

        s_applied_throttle = thr;
        s_applied_brake    = 0u;
        s_applied_steering = steer;
    } else if (s_aeb.brake_pct > 0u) {
        /* ACTIVE braking - short the bridges at the cascade's demanded level.
         * TB6612_BrakeLevel() owns both wheels, so it replaces the mixer for
         * this pass rather than being layered on top of it. Steering is given
         * up for the duration: a differential-drive turn is a wheel-speed
         * difference, which is exactly what a symmetric short removes. */
        TB6612_BrakeLevel(s_aeb.brake_pct);
        DRV_PWM_SetDuty(CAR_DRIVE_TIMER, CAR_DRIVE_CHANNEL, 0u);
        steering_relax(now_ms);

        s_applied_throttle = 0u;
        s_applied_brake    = s_aeb.brake_pct;
        s_applied_steering = 0;
    } else if (s_aeb.inhibit_throttle) {
        /* Intervening, but with nothing left to brake: the tail of the
         * cooldown after the wheels have stopped, and the whole of a PARTIAL
         * stage configured to coast (CAR_AEB_PARTIAL_BRAKE 0).
         *
         * COAST - do not pass this through the mixer. ControlApp_Drive(0,
         * steering) is not "stopped with steering available"; the mixer gives
         * the wheels equal and opposite speeds, so a driver holding steering
         * would have the car spinning on the spot in the middle of an AEB
         * intervention. */
        ControlApp_Stop();
        DRV_PWM_SetDuty(CAR_DRIVE_TIMER, CAR_DRIVE_CHANNEL, 0u);
        steering_relax(now_ms);

        s_applied_throttle = 0u;
        s_applied_brake    = 0u;
        s_applied_steering = 0;
    } else {
        /* Driver has the car: IDLE and WARN. */
        ControlApp_Drive(throttle, steer);
        DRV_PWM_SetDuty(CAR_DRIVE_TIMER, CAR_DRIVE_CHANNEL, thr);

        s_applied_throttle = thr;
        s_applied_brake    = (uint8_t)s_cmd.gw_brake;
        s_applied_steering = steer;
    }

    DRV_GPIO_Write(CAR_REVERSE_LED_PORT, CAR_REVERSE_LED_PIN,
                   reverse ? GPIO_HIGH : GPIO_LOW);

    /* The DRIVER's brake button remains coast-only ("cut throttle"). Active
     * short-braking is the AEB's actuator, deliberately: it is the stage that
     * has a measured reason to demand a specific retarding torque. */
}

/* -------------------------------------------------------------------------- */
/*  Motion estimate                                                           */
/* -------------------------------------------------------------------------- */

/** Degrees per second per raw count, for the range currently configured. */
#if CAR_IMU_ENABLED
static float gyro_dps_per_lsb(void)
{
    switch (MPU6050_GetGyroRange()) {
    case MPU6050_GYRO_FS_250DPS:  return 1.0f / 131.0f;
    case MPU6050_GYRO_FS_1000DPS: return 1.0f / 32.8f;
    case MPU6050_GYRO_FS_2000DPS: return 1.0f / 16.4f;
    case MPU6050_GYRO_FS_500DPS:
    default:                      return 1.0f / 65.5f;
    }
}

/**
 * @brief Sample the gyro, at 100 Hz rather than every pass.
 *
 * A 14-byte burst at 400 kHz costs roughly 400 us. The main loop is free
 * running, so reading on every iteration would spend most of the node's time
 * on a measurement that only changes at the encoder sample rate anyway.
 *
 * A failed read is not immediately fatal - one NAK on a shared bus happens -
 * but IMU_MISS_LIMIT of them in a row means the sensor is gone, and it is
 * better to fall back to the kinematic estimate than to keep publishing a
 * frozen yaw rate that looks live.
 */
/**
 * @brief Try to get a dropped-out IMU back, every IMU_RETRY_MS.
 *
 * Giving up permanently on the first sustained fault was wrong: on a vehicle,
 * a marginal I2C bus drops out and recovers all the time - a connector nudged
 * by vibration, a motor current spike - and losing the sensor for the rest of
 * the run because of a two-second glitch is a far worse outcome than a brief
 * gap in the data.
 *
 * Re-initialising I2C first is the important half: DRV_I2C_Init() now clocks a
 * wedged bus free, which is the single most likely reason a slave stopped
 * answering mid-run.
 *
 * Deliberately cheap - no device reset, no bias recapture - so it costs the
 * control loop a few hundred microseconds even when it fails.
 */
static void imu_retry(uint32_t now_ms)
{
    if (s_imu_ok || (int32_t)(now_ms - s_next_imu_retry_ms) < 0) {
        return;
    }
    s_next_imu_retry_ms = now_ms + IMU_RETRY_MS;

    /* A wrong WHO_AM_I is the wrong part on the bus and will still be wrong in
     * two seconds, so do not retry that case at all. */
    if ((s_imu_who != 0u) && (s_imu_who != MPU6050_WHO_AM_I_VALUE)) {
        return;
    }

    const I2C_Config cfg = {
        .i2c      = CAR_MPU_I2C,
        .speed_hz = CAR_MPU_I2C_HZ,
        .remap    = false,
    };
    if (DRV_I2C_Init(&cfg) != DRV_OK) {
        return;
    }
    if (MPU6050_Resume() != DRV_OK) {
        return;
    }

    s_imu_misses = 0u;
    s_imu_ok     = true;
}

static void imu_poll(uint32_t now_ms)
{
    if (!s_imu_ok || (int32_t)(now_ms - s_next_imu_ms) < 0) {
        return;
    }
    s_next_imu_ms = now_ms + IMU_PERIOD_MS;

    MPU6050_Raw_t raw;
    if (MPU6050_ReadRaw(&raw) != DRV_OK) {
        if (++s_imu_misses >= IMU_MISS_LIMIT) {
            s_imu_ok     = false;
        }
        return;
    }
    s_imu_misses = 0u;

    /* ---- zero-rate offset, gathered incrementally ---------------------- */
    if (!s_bias_done) {
        s_bias_sum_gz += raw.gyro_z;

        if (++s_bias_n >= CAR_MPU_BIAS_SAMPLES) {
            s_gyro_z_bias = (float)s_bias_sum_gz / (float)s_bias_n;
            s_bias_done   = true;
        }

        /* Reporting zero is honest until the offset is known - the car has not
         * moved since boot. */
        s_yaw_imu_mrads = 0.0f;
        return;
    }

    /* ---- yaw rate ------------------------------------------------------ */
    const float dps = ((float)raw.gyro_z - s_gyro_z_bias) * gyro_dps_per_lsb();

    /* rad/s = deg/s x pi/180; x1000 for the milliradian units of the DBC. */
    float mrads = dps * (3.14159265f / 180.0f) * 1000.0f;

    if (CAR_MPU_YAW_INVERT) {
        mrads = -mrads;
    }
    s_yaw_imu_mrads = mrads;
}

#endif /* CAR_IMU_ENABLED */

/**
 * @brief Turn wheel RPM into the body speed and yaw rate that 0x310 carries.
 *
 * Differential drive: forward speed is the mean of the two wheels. Yaw comes
 * from the IMU when one is fitted, and from the wheel-speed difference over
 * the track width when it is not - the kinematic estimate is geometrically
 * exact only while both wheels roll without slipping, so it reads zero for a
 * locked wheel and understates every skid.
 *
 * Speed is still encoder-only. Integrating accelerometer X would need the same
 * bias treatment as the gyro plus a drift correction, and the wheels are the
 * better source while they are rolling.
 */
static void motion_update(void)
{
    MotorEncoder_Process();

    const float l_mmps = MotorEncoder_GetLeftRPM()  * WHEEL_CIRCUM_MM / 60.0f;
    const float r_mmps = MotorEncoder_GetRightRPM() * WHEEL_CIRCUM_MM / 60.0f;

    const float speed = (l_mmps + r_mmps) * 0.5f;

    const float yaw = s_imu_ok
        ? s_yaw_imu_mrads
        : ((r_mmps - l_mmps) / CAR_TRACK_WIDTH_MM) * 1000.0f;

    s_speed_mmps = (int16_t)((speed >  3000.0f) ?  3000.0f :
                             (speed < -3000.0f) ? -3000.0f : speed);
    s_yaw_mrads  = (int16_t)((yaw   >  6000.0f) ?  6000.0f :
                             (yaw   < -6000.0f) ? -6000.0f : yaw);
}

/* -------------------------------------------------------------------------- */
/*  CAN publish / consume                                                     */
/* -------------------------------------------------------------------------- */

static void can_publish(uint32_t id, uint8_t *data, uint8_t *counter)
{
    CAN_TxHeader_t frame;

    E2E_Protect((uint16_t)id, data, 8u, counter);

    frame.std_id = id;
    frame.dlc    = 8u;
    memcpy(frame.data, data, 8u);

    if (!DRV_CAN_Transmit(&frame)) {
        s_tx_dropped++;
    }
}

static void publish_status(void)
{
    struct adas_vc_status_t st = {0};
    uint8_t data[8];

    st.vc_vehicle_state =
        !command_ok()             ? ADAS_VC_STATUS_VC_VEHICLE_STATE_FAILSAFE_CHOICE
      : (s_cmd.gw_armed != 0u)    ? ADAS_VC_STATUS_VC_VEHICLE_STATE_ARMED_CHOICE
      :                             ADAS_VC_STATUS_VC_VEHICLE_STATE_DISARMED_CHOICE;

    /* AEB_State is numbered to match VC_AebState in the DBC, so this is a
     * cast rather than a lookup - see the enum comment in aeb.h. */
    st.vc_aeb_state = (uint8_t)s_aeb.state;

    /* The longitudinal axis goes to whoever actually has authority this cycle.
     * WARN does not count - the driver still has the car. */
    st.vc_arb_winner_long =
        !command_ok()         ? ADAS_VC_STATUS_VC_ARB_WINNER_LONG_FAILSAFE_CHOICE
      : aeb_has_long_axis()   ? ADAS_VC_STATUS_VC_ARB_WINNER_LONG_AEB_CHOICE
      :                         ADAS_VC_STATUS_VC_ARB_WINNER_LONG_DRIVER_CHOICE;
    st.vc_arb_winner_yaw  = command_ok()
                              ? ADAS_VC_STATUS_VC_ARB_WINNER_YAW_DRIVER_CHOICE
                              : ADAS_VC_STATUS_VC_ARB_WINNER_YAW_FAILSAFE_CHOICE;

    st.vc_degraded         = (s_faults != 0u) ? 1u : 0u;
    st.vc_faults           = s_faults;
    st.vc_applied_throttle = s_applied_throttle;
    st.vc_applied_brake    = s_applied_brake;
    st.vc_applied_steering = s_applied_steering;
    st.vc_e2e_err_count    = s_e2e_errors;

    if (adas_vc_status_pack(data, &st, sizeof(data)) == 8) {
        can_publish(ADAS_VC_STATUS_FRAME_ID, data, &s_alv_status);
    }
}

static void publish_motion(void)
{
    struct adas_vc_motion_t mo = {0};
    uint8_t data[8];

    mo.vcm_speed    = s_speed_mmps;
    mo.vcm_yaw_rate = s_yaw_mrads;

    /* Published EVERY cycle, whether or not the AEB is acting - it is the
     * distance at which it WOULD act for the speed currently measured. That
     * makes the ten-run validation protocol scoreable from a bus trace alone:
     * compare VCM_LpbDistance against SF_Range and the point the car actually
     * stopped, with no instrumentation on the car. */
    mo.vcm_lpb_distance = s_aeb.lpb_mm;

    if (adas_vc_motion_pack(data, &mo, sizeof(data)) == 8) {
        can_publish(ADAS_VC_MOTION_FRAME_ID, data, &s_alv_motion);
    }
}

static void publish_heartbeat(uint32_t now_ms)
{
    struct adas_vc_heartbeat_t hb = {0};
    uint8_t data[8];
    uint8_t tec = 0u, rec = 0u;

    DRV_CAN_GetErrorCounters(&tec, &rec);

    hb.hbvc_node_id      = CAR_NODE_ID_VEHICLE;
    hb.hbvc_node_state   = DRV_CAN_IsBusOff()
                             ? ADAS_VC_HEARTBEAT_HBVC_NODE_STATE_FAULT_CHOICE
                             : (s_faults != 0u)
                                 ? ADAS_VC_HEARTBEAT_HBVC_NODE_STATE_DEGRADED_CHOICE
                                 : ADAS_VC_HEARTBEAT_HBVC_NODE_STATE_RUN_CHOICE;
    hb.hbvc_reset_reason = (uint8_t)s_reset_reason;
    hb.hbvc_uptime       = (uint16_t)(now_ms / 1000u);
    hb.hbvc_can_tec      = tec;
    hb.hbvc_can_rec      = rec;

    if (adas_vc_heartbeat_pack(data, &hb, sizeof(data)) == 8) {
        can_publish(ADAS_VC_HEARTBEAT_FRAME_ID, data, &s_alv_hb);
    }
}

static void can_poll(uint32_t now_ms)
{
    CAN_RxHeader_t rx;

    while (DRV_CAN_Receive(&rx)) {
        if (rx.dlc != 8u) {
            continue;
        }

        /* The fused object list. Same E2E discipline as the driver command,
         * but its own receiver: the two are independent streams and sharing a
         * counter would make every frame of one look LOST to the other. */
        if (rx.std_id == ADAS_SENSOR_FRONT_OBJECT_FRAME_ID) {
            const E2E_Status ost =
                E2E_Check((uint16_t)rx.std_id, rx.data, 8u, &s_rx_sf_obj);

            if ((ost == E2E_CRC_ERROR) || (ost == E2E_REPEATED)) {
                if (s_e2e_errors < 255u) {
                    s_e2e_errors++;
                }
                s_faults |= FAULT_E2E;
            s_last_e2e_err_ms = now_ms;
                continue;
            }
            if (ost == E2E_LOST) {
                if (s_e2e_errors < 255u) {
                    s_e2e_errors++;
                }
                s_faults |= FAULT_E2E;
            s_last_e2e_err_ms = now_ms;
            }

            (void)adas_sensor_front_object_unpack(&s_sf_object, rx.data, 8u);
            s_last_sf_obj_ms = now_ms;
            s_seen_sf_obj    = true;
            s_can_rx_object++;
            continue;
        }

        if (rx.std_id != ADAS_GATEWAY_DRIVER_CMD_FRAME_ID) {
            continue;
        }

        /* The three failure modes are NOT equivalent, and treating them alike
         * makes one bad frame cost two:
         *
         *   CRC_ERROR  the bytes cannot be trusted            -> discard
         *   REPEATED   sender is stalled, we already have this -> discard, and
         *              deliberately do NOT refresh the timestamp, so a frozen
         *              gateway still trips the timeout
         *   LOST       earlier frames vanished, but THIS one is CRC-valid and
         *              fresh                                   -> USE IT, flag it
         *
         * Discarding on LOST would throw away good data purely because an
         * earlier frame went missing - and since a discarded frame does not
         * advance the receiver's counter, the next one looks lost too.
         */
        const E2E_Status st = E2E_Check((uint16_t)rx.std_id, rx.data, 8u, &s_rx_cmd);

        if ((st == E2E_CRC_ERROR) || (st == E2E_REPEATED)) {
            if (s_e2e_errors < 255u) {
                s_e2e_errors++;
            }
            s_faults |= FAULT_E2E;
            s_last_e2e_err_ms = now_ms;
            continue;
        }
        if (st == E2E_LOST) {
            if (s_e2e_errors < 255u) {
                s_e2e_errors++;
            }
            s_faults |= FAULT_E2E;
            s_last_e2e_err_ms = now_ms;
            /* fall through - the data is good */
        }

        (void)adas_gateway_driver_cmd_unpack(&s_cmd, rx.data, 8u);
        s_last_cmd_ms = now_ms;
        s_have_cmd    = true;
    }
}

static void faults_update(void)
{
    if (DRV_CAN_IsBusOff()) {
        s_faults |= FAULT_BUS_OFF;
    } else {
        s_faults &= (uint8_t)~FAULT_BUS_OFF;
    }

    if (s_have_cmd && DRV_SysTick_Elapsed(s_last_cmd_ms, CAR_CAN_CMD_TIMEOUT_MS)) {
        s_faults |= FAULT_GW_TIMEOUT;
    } else if (command_ok()) {
        s_faults &= (uint8_t)~FAULT_GW_TIMEOUT;
    }

    /* The front object list. Only a fault once one has actually been seen -
     * a car built without the sensor node must still drive, and a node that
     * has never spoken is a configuration, not a failure. Once it HAS spoken,
     * going quiet is a genuine fault: it is the AEB's only input. */
    if (s_seen_sf_obj &&
        DRV_SysTick_Elapsed(s_last_sf_obj_ms, CAR_SF_OBJECT_TIMEOUT_MS)) {
        s_faults |= FAULT_FRONT_TIMEOUT;
    } else if (s_seen_sf_obj) {
        s_faults &= (uint8_t)~FAULT_FRONT_TIMEOUT;
    }

    /* FAULT_E2E is set by can_poll() on any CRC, repeat or gap, and NOTHING
     * used to clear it. One corrupted frame in a whole run therefore latched
     * VC_Degraded on permanently: the station showed the car as faulted for
     * the rest of the session, and every later genuine fault was hidden
     * underneath a flag that had already been raised.
     *
     * It now clears once a full timeout's worth of frames has arrived clean,
     * which is long enough that a real recurring problem stays visible and
     * short enough that a single glitch does not poison the run.
     *
     * The saturating s_e2e_errors COUNT is deliberately not reset - the flag
     * says "happening now", the count says "how often", and the ten-run
     * protocol wants the second. */
    if (command_ok() &&
        DRV_SysTick_Elapsed(s_last_e2e_err_ms, CAR_CAN_CMD_TIMEOUT_MS)) {
        s_faults &= (uint8_t)~FAULT_E2E;
    }
}

/* -------------------------------------------------------------------------- */
/*  AEB                                                                       */
/* -------------------------------------------------------------------------- */

/** @brief True while the fused object is fresh enough to act on. */
static bool sf_object_ok(void)
{
    return s_seen_sf_obj &&
           !DRV_SysTick_Elapsed(s_last_sf_obj_ms, CAR_SF_OBJECT_TIMEOUT_MS);
}

/**
 * @brief Run the AEB cascade for this cycle.
 *
 * Called every pass, not only when a 0x100 arrives: the latch timer and the
 * de-escalation hysteresis both have to keep running while the object is
 * stale, and the whole point of the cooldown is that it survives its own
 * trigger condition disappearing.
 */
static void aeb_update(uint32_t now_ms)
{
    AEB_Input in;

    in.now_ms          = now_ms;
    in.speed_mmps      = s_speed_mmps;      /* encoders - sets the threshold
                                             * AND says when braking is done */
    in.yaw_mrads       = s_yaw_mrads;       /* measured rotation - the backstop*/
    /* The RAW command, not the shaped value: this is about the driver's
     * intent to turn, and the raw stick expresses it before the slew limiter
     * has let the wheels act on it. */
    in.steer_pct       = (int8_t)s_cmd.gw_steering;
    in.range_mm        = (uint16_t)s_sf_object.sf_range;
    in.range_rate_mmps = (int16_t)s_sf_object.sf_range_rate;
    in.track_age       = (uint8_t)s_sf_object.sf_track_age;

    /* Three things must all hold, and the freshness check is the one that is
     * easy to forget: a stale frame still unpacks perfectly. */
    in.object_valid = sf_object_ok() &&
                      (s_sf_object.sf_status ==
                       ADAS_SENSOR_FRONT_OBJECT_SF_STATUS_VALID_CHOICE);

    AEB_Update(&in, &s_aeb);

    /* ---- diagnostic ------------------------------------------------------
     * Live fields first, then the peak-hold. */
    g_vc_debug.seq             = ++s_loop_passes;
    g_vc_debug.can_rx_object   = s_can_rx_object;
    g_vc_debug.obj_age_ms      = s_seen_sf_obj ? (now_ms - s_last_sf_obj_ms)
                                               : 0xFFFFFFFFu;
    g_vc_debug.e2e_errors      = s_e2e_errors;
    g_vc_debug.speed_mmps      = s_speed_mmps;
    g_vc_debug.yaw_mrads       = s_yaw_mrads;
    g_vc_debug.range_mm        = in.range_mm;
    g_vc_debug.range_rate_mmps = in.range_rate_mmps;
    g_vc_debug.aeb_state       = (uint32_t)s_aeb.state;
    g_vc_debug.aeb_block       = (uint32_t)s_aeb.block;
    g_vc_debug.lpb_mm          = s_aeb.lpb_mm;

    if (s_aeb.brake_pct > 0u) {
        g_vc_debug.brake_cycles = ++s_brake_cycles;
    }
    if ((uint32_t)s_aeb.state > g_vc_debug.worst_state) {
        g_vc_debug.worst_state = (uint32_t)s_aeb.state;
    }

    /* Latch the approach's worst moment. Gated on moving FORWARD and on a
     * believable object, so a car sitting still against a wall - or one
     * reversing away from it - does not overwrite the evidence from the run
     * that actually mattered. */
    if (in.object_valid && (in.speed_mmps > 0) &&
        ((g_vc_debug.closest_range == 0u) ||
         ((uint32_t)in.range_mm < g_vc_debug.closest_range))) {
        g_vc_debug.closest_range = in.range_mm;
        g_vc_debug.closest_speed = in.speed_mmps;
        g_vc_debug.closest_lpb   = s_aeb.lpb_mm;
        g_vc_debug.closest_block = (uint32_t)s_aeb.block;
        g_vc_debug.closest_age   = in.track_age;
    }

    /* ---- CAN health, peak-held ---------------------------------------- */
    {
        uint8_t tec = 0u, rec = 0u;
        static bool s_was_bus_off;
        const bool off = DRV_CAN_IsBusOff();

        DRV_CAN_GetErrorCounters(&tec, &rec);
        if ((uint32_t)tec > g_vc_debug.max_tec) { g_vc_debug.max_tec = tec; }
        if ((uint32_t)rec > g_vc_debug.max_rec) { g_vc_debug.max_rec = rec; }

        /* Count the EDGE. ABOM lets the peripheral recover on its own, so a
         * whole bus-off can happen between two passes and leave nothing behind
         * unless the transition itself is recorded. */
        if (off && !s_was_bus_off) {
            g_vc_debug.bus_off_events++;
        }
        s_was_bus_off = off;
    }

    /* Longest gap between accepted commands, and how often that gap was long
     * enough to have caused a FAILSAFE. */
    if (s_have_cmd) {
        const uint32_t gap = now_ms - s_last_cmd_ms;
        static bool s_was_failsafe;
        const bool fs = !command_ok();

        if (gap > g_vc_debug.max_cmd_gap_ms) {
            g_vc_debug.max_cmd_gap_ms = gap;
        }
        if (fs && !s_was_failsafe) {
            g_vc_debug.failsafe_events++;
        }
        s_was_failsafe = fs;
    }

    g_vc_debug.boot_count   = s_boot_count;
    g_vc_debug.reset_reason = (uint32_t)s_reset_reason;
    g_vc_debug.uptime_ms    = now_ms;

    /* Longest single pass of the run. A pass longer than the command timeout
     * puts this node into FAILSAFE on its own - see vc_debug.h. */
    if (s_prev_pass_ms != 0u) {
        const uint32_t pass = now_ms - s_prev_pass_ms;
        if (pass > g_vc_debug.max_loop_ms) {
            g_vc_debug.max_loop_ms = pass;
        }
        if (pass > 20u) {
            g_vc_debug.slow_passes = ++s_slow_passes;
        }
    }
    s_prev_pass_ms = now_ms;

    g_vc_debug.magic = VC_DEBUG_MAGIC;   /* last: the struct is now valid */
}

/* -------------------------------------------------------------------------- */
/*  Initialisation                                                            */
/* -------------------------------------------------------------------------- */

#if CAR_IMU_ENABLED
/**
 * @brief Bring up I2C2 and the MPU6050, then capture the gyro's zero offset.
 *
 * Every failure path here simply leaves s_imu_ok false, and the node runs on
 * the kinematic yaw estimate exactly as it did before the sensor existed. An
 * absent IMU must not stop a car that can otherwise drive safely.
 */
static void imu_init(void)
{
    const I2C_Config cfg = {
        .i2c      = CAR_MPU_I2C,
        .speed_hz = CAR_MPU_I2C_HZ,
        .remap    = false,          /* I2C1 only; ignored for I2C2 */
    };

    if (DRV_I2C_Init(&cfg) != DRV_OK) {
        return;
    }

    /* Ask WHO_AM_I directly before MPU6050_Init(), purely so the failure can be
     * told apart. A bus that NAKs (nothing there, SDA/SCL swapped, no pull-ups,
     * wrong AD0 strap) looks nothing like a part that answers with the wrong
     * ID (an MPU6500/9250 clone sold as a 6050), and the two need different
     * fixes. Without this both just came back "init failed". */
    if (MPU6050_ReadWhoAmI(&s_imu_who) != DRV_OK) {
        return;
    }

    if (MPU6050_Init() != DRV_OK) {
        return;
    }

    /* The zero-rate offset is gathered by imu_poll() over the next second or
     * so, while the car is still sitting where it booted. Until it is ready
     * the node reports zero yaw, which is true anyway. */
    s_bias_sum_gz = 0;
    s_bias_n      = 0u;
    s_bias_done   = false;

    s_imu_ok      = true;
}

#else
/* IMU compiled out - see CAR_IMU_ENABLED in vehicle_config.h. The call sites
 * stay put and evaluate to nothing, so the loop keeps one shape either way and
 * there is no second version of it to keep in step. */
#define imu_init()   ((void)0)
#define imu_poll(t)  ((void)(t))
#define imu_retry(t) ((void)(t))
#endif /* CAR_IMU_ENABLED */

static void board_init(void)
{
    /* FIRST: reads and clears RCC_CSR. Anything that reconfigures the
     * clock tree must not run before this. */
    s_reset_reason = DRV_Clock_ResetReason();

    /* Count reboots that did NOT lose power. Garbage on a cold start, so the
     * magic decides which of the two this is. */
    if (s_boot_magic != BOOT_MAGIC) {
        s_boot_magic = BOOT_MAGIC;
        s_boot_count = 1u;          /* cold start - battery just connected */
    } else {
        s_boot_count++;             /* warm reset - RAM kept its contents  */
    }

    s_clock_ok = (DRV_Clock_Init72MHz(CAR_HSE_HZ) == DRV_OK);
    if (!s_clock_ok) {
        s_faults |= FAULT_CLOCK;
    }
    DRV_Delay_Init();
    DRV_SysTick_Init(CAR_TICK_IRQ_PRIORITY);

    DRV_GPIO_Write(CAR_LED_STATUS_PORT, CAR_LED_STATUS_PIN,
                   CAR_LED_ACTIVE_LOW ? GPIO_HIGH : GPIO_LOW);
    DRV_GPIO_Init(CAR_LED_STATUS_PORT, CAR_LED_STATUS_PIN,
                  GPIO_MODE_OUT_2M, GPIO_CNF_OUT_PP, GPIO_PULL_NONE);

    DRV_GPIO_InitOutput(CAR_REVERSE_LED_PORT, CAR_REVERSE_LED_PIN, GPIO_LOW);

    /* Drive-indicator PWM, starts at 0 %. Shares TIM3 with the motors at the
     * same 20 kHz, so the order of the two is immaterial. */
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
     * PA4/STBY - do not also drive it externally. */
    ControlApp_Init();
    MotorEncoder_Init();

    /* AEB before CAN, so the first 0x100 that arrives meets an initialised
     * state machine rather than zeroed config with a zero deceleration. */
    AEB_ConfigDefaults(&s_aeb_cfg);
    CAR_AEB_LoadConfig(&s_aeb_cfg);
    AEB_Init(&s_aeb_cfg);

    /* CAN last: bringing it up before the motors are in a known-stopped state
     * would let a command arrive with the outputs unconfigured. */
    s_can_ok = DRV_CAN_Init(CAR_CAN_MODE);

    /* ANNOUNCE PRESENCE THE INSTANT THE BUS IS UP, before the IMU below.
     *
     * The gateway only marks this node present once it has seen a heartbeat,
     * and the station renders its absence as "NO CAR". Leaving the first
     * heartbeat until the main loop meant it went out AFTER imu_init()'s
     * ~240 ms of blocking bring-up, so every power-on painted NO CAR for a
     * quarter of a second - truthfully, but for no good reason.
     *
     * Bringing the IMU up after CAN already followed this principle. It just
     * was not carried far enough: what matters is not when the peripheral is
     * initialised but when this node first SAYS SOMETHING, and that had stayed
     * behind the optional sensor. */
    if (s_can_ok) {
        publish_heartbeat(DRV_SysTick_GetTick());
    }

    /* IMU AFTER the bus, deliberately. It blocks for ~240 ms on a good day and
     * far longer against a wedged I2C bus, where every transaction runs to its
     * timeout. An optional diagnostic sensor must not sit in front of the bus
     * that carries this node's liveness - a miswired MPU6050 should never be
     * able to make the vehicle node look dead. */
    imu_init();

    /* A SECOND heartbeat, because the gateway drops a node after 300 ms and
     * imu_init() can spend most of that on its own. Without this the node can
     * announce itself, disappear while the IMU comes up, and be declared
     * absent before the main loop has published anything - which looks
     * identical to the fault this is meant to rule out. */
    if (s_can_ok) {
        publish_heartbeat(DRV_SysTick_GetTick());
    }
}

int main(void)
{
    board_init();

    s_next_status_ms = DRV_SysTick_GetTick();
    s_next_hb_ms     = s_next_status_ms;

    forever {
        const uint32_t now = DRV_SysTick_GetTick();

        if (s_can_ok) {
            can_poll(now);
        }

        faults_update();
        imu_retry(now);
        imu_poll(now);
        motion_update();

        /* AFTER motion_update(): the cascade's threshold is a function of the
         * speed measured this pass, so running it on last pass's speed would
         * brake against a stale number. BEFORE outputs_apply(), which applies
         * what it decides. */
        aeb_update(now);

        /* Unconditional, every iteration: outputs_apply() re-evaluates the
         * timeout and the armed bit, so a link that dies between frames stops
         * the car without waiting for the next one to arrive. */
        outputs_apply(now);

        /* TRANSMIT BUDGET: bxCAN has three TX mailboxes and this node runs with
         * NART, so a frame that finds them all full is DROPPED, not retried.
         * Queueing four in one pass therefore loses one silently.
         *
         * The heartbeat period is an exact multiple of the status period, so
         * the two schedules coincide on every heartbeat - there is no luck
         * involved, the same frame loses every time. Two rules keep that from
         * happening: the heartbeat goes FIRST because it is the node's
         * liveness signal, leaving the status pair behind it. Three frames
         * against three mailboxes - no headroom, so anything added here needs
         * its own phase. */
        if (s_can_ok) {
            if ((int32_t)(now - s_next_hb_ms) >= 0) {
                publish_heartbeat(now);
                s_next_hb_ms += CAR_CAN_HEARTBEAT_MS;
            }
            if ((int32_t)(now - s_next_status_ms) >= 0) {
                publish_status();
                publish_motion();
                s_next_status_ms += CAR_CAN_STATUS_PERIOD_MS;
            }
        }

        status_led_update();
    }
}
