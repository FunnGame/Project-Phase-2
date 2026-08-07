#include "motor_encoder.h"

#include "drv_encoder.h"
#include "drv_timer.h"
#include "car_config.h"


#define ENC_RPM_SCALE  (60.0f * (float)CAR_ENC_SAMPLE_HZ)


static volatile int16_t  s_delta_l;
static volatile int16_t  s_delta_r;
static volatile int32_t  s_ticks_l;
static volatile int32_t  s_ticks_r;

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
        .mode   = ENCODER_MODE_BOTH,    
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

    s_prev_l = DRV_Encoder_Get(CAR_ENC_L_TIMER);
    s_prev_r = DRV_Encoder_Get(CAR_ENC_R_TIMER);

    MotorEncoder_ResetTicks();

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

    do {
        seq = s_sample_seq;
        dl  = s_delta_l;
        dr  = s_delta_r;
    } while (seq != s_sample_seq);

    if (seq == s_seen_seq) {
        return;                             
    }
    s_seen_seq = seq;

    s_rpm_l = ((float)dl / CAR_ENC_COUNTS_PER_REV) * ENC_RPM_SCALE;
    s_rpm_r = ((float)dr / CAR_ENC_COUNTS_PER_REV) * ENC_RPM_SCALE;
}

int16_t MotorEncoder_GetLeftDelta(void)  { return s_delta_l; }
int16_t MotorEncoder_GetRightDelta(void) { return s_delta_r; }

float MotorEncoder_GetLeftRPM(void)  { return s_rpm_l; }
float MotorEncoder_GetRightRPM(void) { return s_rpm_r; }


int32_t MotorEncoder_GetLeftTicks(void)  { return s_ticks_l; }
int32_t MotorEncoder_GetRightTicks(void) { return s_ticks_r; }

void MotorEncoder_ResetTicks(void)
{
    s_ticks_l = 0;
    s_ticks_r = 0;
}