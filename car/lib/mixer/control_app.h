#ifndef CONTROL_APP_H
#define CONTROL_APP_H

#include <stdint.h>

/**
 ******************************************************************************
 * @file    control_app.h
 * @brief   Differential-drive mixer.
 *
 * Turns ONE motion intent — a throttle and a steering value — into two signed
 * wheel-speed commands and drives the TB6612 motors.
 *
 ******************************************************************************
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Bring up the motor driver and leave the car stopped.
 *
 * Call once after the clock is configured — the motor driver derives its PWM
 * frequency from the real timer clock.
 */
void ControlApp_Init(void);

/**
 * @brief Mix one intent onto the two wheels and drive the motors.
 * @param throttle -100..+100. Positive drives forward, negative reverse.
 * @param steering -100..+100. Positive turns right, negative turns left.
 *
 * Each wheel speed is throttle +/- steering, clamped to the motor range — so
 * at high throttle a steering command trades some of the inside wheel's speed
 * for the turn.
 */
void ControlApp_Drive(int8_t throttle, int8_t steering);

/** @brief Coast both wheels. Removes drive; does not actively brake. */
void ControlApp_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_APP_H */
