/**
 ******************************************************************************
 * @file    tb6612_motor.h
 * @brief   TB6612FNG dual H-bridge driver for the car's two drive wheels.
 *
 * Each motor takes three signals: a PWM pin setting the magnitude and two
 * direction pins (IN1/IN2) setting the sign. A shared STBY pin gates both
 * bridges. All pins, the timer, the channels and the PWM frequency come from
 * vehicle_config.h.
 *
 * Two levels of API:
 *   - TB6612_SetMotor()  one wheel, signed speed. The primitive; this is what
 *                        a differential-drive mixer or the ADAS arbiter wants.
 *   - TB6612_Drive()     both wheels via a named manoeuvre. Convenient for
 *                        bring-up and manual teleoperation.
 *
 * See MOTORS.md in this folder for the wiring table, the TB6612 truth table
 * and worked examples.
 ******************************************************************************
 */
#ifndef TB6612_MOTOR_H_
#define TB6612_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Which wheel. */
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
} Motor_Side_t;

/** @brief Named manoeuvres for TB6612_Drive(). */
typedef enum {
    CAR_STOP = 0,    /**< Coast: outputs off, wheels free to spin.        */
    CAR_FORWARD,     /**< Both wheels forward.                            */
    CAR_BACKWARD,    /**< Both wheels backward.                           */
    CAR_TURN_LEFT,   /**< Spin in place: left back, right forward.        */
    CAR_TURN_RIGHT,  /**< Spin in place: left forward, right back.        */
} Car_Direction_t;

/**
 * @brief Configure the direction pins, STBY and both PWM channels.
 *
 * Leaves the car stopped and takes the driver out of standby. Call after the
 * clock is up (the PWM frequency is derived from the real timer clock).
 */
void TB6612_Init(void);

/**
 * @brief Drive one wheel.
 * @param side  Which wheel.
 * @param speed -100..+100. Positive is forward, negative reverse, 0 coasts.
 *              Values outside the range are clamped.
 *
 * The per-wheel CAR_MOTOR_*_INVERT flags in vehicle_config.h are applied here, so
 * "positive is forward" holds however the motor leads happen to be soldered.
 */
void TB6612_SetMotor(Motor_Side_t side, int8_t speed);

/**
 * @brief Drive both wheels via a named manoeuvre.
 * @param dir         Manoeuvre.
 * @param left_speed  Magnitude for the left wheel, 0..100 (clamped).
 * @param right_speed Magnitude for the right wheel, 0..100 (clamped).
 *
 * Both speeds are magnitudes — @p dir supplies the sign for each wheel. For
 * CAR_STOP both are ignored.
 */
void TB6612_Drive(Car_Direction_t dir, uint8_t left_speed, uint8_t right_speed);

/** @brief Coast: PWM to 0 and both bridges to their off state. */
void TB6612_Stop(void);

/**
 * @brief Short-brake both motors (IN1 = IN2 = 1, full PWM).
 *
 * Shorts each motor's terminals so it resists rotation, which stops the car
 * far faster than coasting. This is the actuator the AEB full-braking stage
 * will use — TB6612_Stop() only removes drive.
 */
void TB6612_Brake(void);

/**
 * @brief Enable or disable the driver's outputs.
 * @param enabled true to run, false for standby (both bridges high-impedance).
 *
 * Standby overrides everything, so this is the cheapest hardware-level kill
 * switch for the disarmed and failsafe states.
 */
void TB6612_SetStandby(bool enabled);

#ifdef __cplusplus
}
#endif

#endif /* TB6612_MOTOR_H_ */
