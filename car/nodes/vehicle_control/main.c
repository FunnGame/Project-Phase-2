/**
 ******************************************************************************
 * @file    main.c
 * @brief   Vehicle-control node: the only node that touches the actuators.
 *
 * Runs on an STM32F103. Took the motors and encoders off the gateway when the
 * car was split across three MCUs.
 *
 *   1. brings up the clock, tick, status LED, CAN, motors and encoders,
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

#include "vehicle_config.h"
#include "control_app.h"    /* differential-drive mixer (car/lib/mixer)      */
#include "motor_encoder.h"  /* wheel speed (car/devices/tb6612)              */

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
static uint8_t  s_faults;            /* VC_Faults bitfield                   */
static uint8_t  s_e2e_errors;        /* saturating, published in 0x300       */
static uint32_t s_tx_dropped;

/* ---- What we actually applied, for 0x300 --------------------------------- */
static uint8_t  s_applied_throttle;
static uint8_t  s_applied_brake;
static int8_t   s_applied_steering;

/* ---- Measured motion, for 0x310 ------------------------------------------ */
static int16_t  s_speed_mmps;
static int16_t  s_yaw_mrads;

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
 * @brief Apply the arbitrated command to the motors.
 *
 * Today there is nothing to arbitrate against - the sensor node is not
 * integrated, so the driver's intent is the only bidder. When AEB arrives it
 * bids here, and the winner is recorded in VC_ArbWinnerLong.
 */
static void outputs_apply(void)
{
    /* Safety gate: a disarmed car ignores drive commands entirely. */
    if (!command_ok() || (s_cmd.gw_armed == 0u)) {
        outputs_stop();
        return;
    }

    /* The frame carries throttle as an unsigned magnitude plus a reverse bit;
     * the mixer wants a signed throttle. Brake wins over throttle - pressing
     * both stops rather than drives. */
    const bool reverse = (s_cmd.gw_reverse != 0u);
    const uint8_t thr  = (uint8_t)s_cmd.gw_throttle;
    int8_t throttle = (s_cmd.gw_brake > 0u) ? 0
                    : reverse ? -(int8_t)thr
                    :            (int8_t)thr;

    ControlApp_Drive(throttle, (int8_t)s_cmd.gw_steering);

    DRV_PWM_SetDuty(CAR_DRIVE_TIMER, CAR_DRIVE_CHANNEL,
                    (s_cmd.gw_brake > 0u) ? 0u : thr);
    DRV_GPIO_Write(CAR_REVERSE_LED_PORT, CAR_REVERSE_LED_PIN,
                   reverse ? GPIO_HIGH : GPIO_LOW);

    s_applied_throttle = (s_cmd.gw_brake > 0u) ? 0u : thr;
    s_applied_brake    = (uint8_t)s_cmd.gw_brake;
    s_applied_steering = (int8_t)s_cmd.gw_steering;

    /* Brake is "cut throttle" (coast) for now. Active braking via TB6612_Brake()
     * is deliberately deferred to the AEB stage, where the warning/partial/full
     * cascade decides when to short the motors. */
}

/* -------------------------------------------------------------------------- */
/*  Motion estimate                                                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Turn wheel RPM into the body speed and yaw rate that 0x310 carries.
 *
 * Differential drive: forward speed is the mean of the two wheels, yaw rate is
 * their difference over the track width. Both come from the encoders alone -
 * there is no IMU on this node yet, so yaw is a kinematic estimate and will
 * read zero for a skidding wheel.
 */
static void motion_update(void)
{
    MotorEncoder_Process();

    const float l_mmps = MotorEncoder_GetLeftRPM()  * WHEEL_CIRCUM_MM / 60.0f;
    const float r_mmps = MotorEncoder_GetRightRPM() * WHEEL_CIRCUM_MM / 60.0f;

    const float speed = (l_mmps + r_mmps) * 0.5f;
    /* rad/s = (v_r - v_l) / track; x1000 for the milliradian units of the DBC */
    const float yaw   = ((r_mmps - l_mmps) / CAR_TRACK_WIDTH_MM) * 1000.0f;

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

    /* No sensor node yet, so AEB has no input and never bids. */
    st.vc_aeb_state       = ADAS_VC_STATUS_VC_AEB_STATE_IDLE_CHOICE;
    st.vc_arb_winner_long = command_ok()
                              ? ADAS_VC_STATUS_VC_ARB_WINNER_LONG_DRIVER_CHOICE
                              : ADAS_VC_STATUS_VC_ARB_WINNER_LONG_FAILSAFE_CHOICE;
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
    /* No AEB yet, so no last-point-to-brake distance to report. */
    mo.vcm_lpb_distance = 0u;

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
    hb.hbvc_reset_reason = ADAS_VC_HEARTBEAT_HBVC_RESET_REASON_POWER_ON_CHOICE;
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
        if ((rx.std_id != ADAS_GATEWAY_DRIVER_CMD_FRAME_ID) || (rx.dlc != 8u)) {
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
            continue;
        }
        if (st == E2E_LOST) {
            if (s_e2e_errors < 255u) {
                s_e2e_errors++;
            }
            s_faults |= FAULT_E2E;
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

    /* The front sensor is not integrated, so its absence is expected rather
     * than a fault. FAULT_FRONT_TIMEOUT starts being set when 0x100 is
     * consumed. */
}

/* -------------------------------------------------------------------------- */
/*  Initialisation                                                            */
/* -------------------------------------------------------------------------- */

static void board_init(void)
{
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

    /* CAN last: bringing it up before the motors are in a known-stopped state
     * would let a command arrive with the outputs unconfigured. */
    s_can_ok = DRV_CAN_Init(CAR_CAN_MODE);
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
        motion_update();

        /* Unconditional, every iteration: outputs_apply() re-evaluates the
         * timeout and the armed bit, so a link that dies between frames stops
         * the car without waiting for the next one to arrive. */
        outputs_apply();

        if (s_can_ok) {
            if ((int32_t)(now - s_next_status_ms) >= 0) {
                publish_status();
                publish_motion();
                s_next_status_ms += CAR_CAN_STATUS_PERIOD_MS;
            }
            if ((int32_t)(now - s_next_hb_ms) >= 0) {
                publish_heartbeat(now);
                s_next_hb_ms += CAR_CAN_HEARTBEAT_MS;
            }
        }

        status_led_update();
    }
}
