/**
 ******************************************************************************
 * @file    motor_encoder.h
 * @brief   Quadrature wheel-encoder sampling and speed estimation.
 *
 * Wraps the two hardware encoder timers (drv_encoder) in a fixed-rate sampler:
 * a timer interrupt latches both counters every CAR_ENC_SAMPLE_HZ, and the
 * main loop turns the latched deltas into RPM.
 *
 * The split matters. The F103 has no FPU, so the RPM arithmetic is soft-float
 * library calls; running them in the ISR would add hundreds of cycles of
 * interrupt latency to a 100 Hz interrupt, and would let a reader in the main
 * loop see a torn 32-bit float mid-update. The ISR therefore does integer work
 * only.
 *
 * Pins, timers, gearing and the sample rate all come from vehicle_config.h.
 ******************************************************************************
 */
#ifndef MOTOR_ENCODER_H_
#define MOTOR_ENCODER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start both encoders and the sampling timer.
 *
 * Call after the clock is configured — the sample rate is derived from the
 * timer's real kernel clock.
 */
void MotorEncoder_Init(void);

/**
 * @brief Convert the newest sample into RPM. Call from the main loop.
 *
 * Cheap and safe to call every iteration: it returns immediately unless the
 * ISR has produced a new sample since last time.
 */
void MotorEncoder_Process(void);

/** @brief Signed encoder counts in the most recent sample window. */
int16_t MotorEncoder_GetLeftDelta(void);
int16_t MotorEncoder_GetRightDelta(void);

/**
 * @brief Output-shaft speed in RPM, positive forward.
 *
 * Updated by MotorEncoder_Process(), so it is only as fresh as the last call.
 */
float MotorEncoder_GetLeftRPM(void);
float MotorEncoder_GetRightRPM(void);

/**
 * @brief Cumulative signed counts since the last reset — wheel odometry.
 *
 * 32-bit and accumulated from the deltas, so unlike the raw 16-bit timer
 * counter it does not wrap every 65536 counts (~11 output revolutions).
 */
int32_t MotorEncoder_GetLeftTicks(void);
int32_t MotorEncoder_GetRightTicks(void);

/** @brief Zero both odometry accumulators. */
void MotorEncoder_ResetTicks(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_ENCODER_H_ */
