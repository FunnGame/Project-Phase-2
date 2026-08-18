/**
 ******************************************************************************
 * @file    motor_encoder.c
 * @brief   Wheel-encoder sampling and speed estimation implementation.
 ******************************************************************************
 */
#include "motor_encoder.h"

#include "drv_encoder.h"
#include "drv_timer.h"
#include "vehicle_config.h"

/* Counts -> RPM: (counts / counts_per_rev) * samples_per_second * 60. */
#define ENC_RPM_SCALE  (60.0f * (float)CAR_ENC_SAMPLE_HZ)

/* ---- Written by the ISR, read by the main loop --------------------------- */
static volatile int16_t  s_delta_l;
static volatile int16_t  s_delta_r;
static volatile int32_t  s_ticks_l;
static volatile int32_t  s_ticks_r;
/* Bumped once per sample. The main loop compares it against its own copy to
 * detect new data, and re-reads if it changes mid-read (a minimal seqlock —
 * the two deltas must come from the same sample to be consistent). */
static volatile uint32_t s_sample_seq;

/* ---- ISR-private ---------------------------------------------------------- */
static uint16_t s_prev_l;
static uint16_t s_prev_r;

/* ---- Main-loop-private ---------------------------------------------------- */
static uint32_t s_seen_seq;
static float    s_rpm_l;
static float    s_rpm_r;

/* -------------------------------------------------------------------------- */

static void encoder_sample_cb(TIM_TypeDef *tim, void *ctx)
{
    DRV_UNUSED(tim);
    DRV_UNUSED(ctx);

    /* Integer work only — see the note in motor_encoder.h. GetDelta handles
     * the 16-bit wrap in both directions. */
    const int16_t dl = DRV_Encoder_GetDelta(CAR_ENC_L_TIMER, &s_prev_l);
    const int16_t dr = DRV_Encoder_GetDelta(CAR_ENC_R_TIMER, &s_prev_r);

    s_delta_l = dl;
    s_delta_r = dr;
    s_ticks_l += dl;
    s_ticks_r += dr;

    s_sample_seq++;
}

void MotorEncoder_Init(void)
{
    const Encoder_Config left = {
        .tim    = CAR_ENC_L_TIMER,
        .port_a = CAR_ENC_L_PORT_A, .pin_a = CAR_ENC_L_PIN_A,
        .port_b = CAR_ENC_L_PORT_B, .pin_b = CAR_ENC_L_PIN_B,
        .arr    = 0xFFFFu,
        .mode   = ENCODER_MODE_BOTH,      /* 4x — matches CAR_ENC_EDGE_MULT */
        .invert = CAR_ENC_L_INVERT,
    };
    const Encoder_Config right = {
        .tim    = CAR_ENC_R_TIMER,
        .port_a = CAR_ENC_R_PORT_A, .pin_a = CAR_ENC_R_PIN_A,
        .port_b = CAR_ENC_R_PORT_B, .pin_b = CAR_ENC_R_PIN_B,
        .arr    = 0xFFFFu,
        .mode   = ENCODER_MODE_BOTH,
        .invert = CAR_ENC_R_INVERT,
    };
    (void)DRV_Encoder_Init(&left);
    (void)DRV_Encoder_Init(&right);

    /* Seed the previous counts so the first sample is a true delta rather than
     * whatever the counters happen to hold. */
    s_prev_l = DRV_Encoder_Get(CAR_ENC_L_TIMER);
    s_prev_r = DRV_Encoder_Get(CAR_ENC_R_TIMER);

    MotorEncoder_ResetTicks();

    /* Sample rate in hertz, not a prescaler/period pair: correct at 8 MHz HSI
     * and at 72 MHz PLL alike. */
    if (DRV_Timer_InitHz(CAR_ENC_SAMPLE_TIMER, CAR_ENC_SAMPLE_HZ) == DRV_OK) {
        (void)DRV_Timer_AttachCallback(CAR_ENC_SAMPLE_TIMER, encoder_sample_cb,
                                       NULL, CAR_ENC_SAMPLE_IRQ_PRIORITY);
        DRV_Timer_Start(CAR_ENC_SAMPLE_TIMER);
    }
}

void MotorEncoder_Process(void)
{
    uint32_t seq;
    int16_t dl, dr;

    /* Retry until the sample did not change underneath us. */
    do {
        seq = s_sample_seq;
        dl  = s_delta_l;
        dr  = s_delta_r;
    } while (seq != s_sample_seq);

    if (seq == s_seen_seq) {
        return;                             /* nothing new since last call */
    }
    s_seen_seq = seq;

    s_rpm_l = ((float)dl / CAR_ENC_COUNTS_PER_REV) * ENC_RPM_SCALE;
    s_rpm_r = ((float)dr / CAR_ENC_COUNTS_PER_REV) * ENC_RPM_SCALE;
}

int16_t MotorEncoder_GetLeftDelta(void)  { return s_delta_l; }
int16_t MotorEncoder_GetRightDelta(void) { return s_delta_r; }

float MotorEncoder_GetLeftRPM(void)  { return s_rpm_l; }
float MotorEncoder_GetRightRPM(void) { return s_rpm_r; }

/* 32-bit aligned loads are atomic on the Cortex-M3, so these need no guard. */
int32_t MotorEncoder_GetLeftTicks(void)  { return s_ticks_l; }
int32_t MotorEncoder_GetRightTicks(void) { return s_ticks_r; }

void MotorEncoder_ResetTicks(void)
{
    s_ticks_l = 0;
    s_ticks_r = 0;
}
