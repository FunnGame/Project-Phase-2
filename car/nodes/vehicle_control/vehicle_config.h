/**
 ******************************************************************************
 * @file    vehicle_config.h
 * @brief   Board configuration for the VEHICLE-CONTROL node
 *          (STM32F103C8, node 2).
 *
 * Edit pins, ports, peripheral instances and vehicle geometry HERE — main.c
 * hard-codes nothing, so re-wiring never means hunting through the code.
 *
 * This is the only node that can move the car: it owns the TB6612, both wheel
 * encoders and the indicator LEDs. It has no radio. Its counterpart is
 * car/nodes/gateway/gateway_config.h.
 *
 * ALSO READ BY car/devices/tb6612 (tb6612_motor, motor_encoder), which take
 * their pins from the CAR_MOTOR_* and CAR_ENC_* macros below. Those modules
 * only ever build into this node.
 ******************************************************************************
 */
#ifndef VEHICLE_CONFIG_H_
#define VEHICLE_CONFIG_H_

#include "drv_common.h"
#include "drv_gpio.h"
#include "drv_pwm.h"
#include "drv_encoder.h"
#include "drv_can.h"
#include "adas.h"         /* cycle times come from the DBC, not from here */

/* ===== System clock ======================================================= */
/* HSE crystal frequency in Hz. */
#define CAR_HSE_HZ              8000000u

/* ===== Timer budget =======================================================
 *   TIM1  left wheel encoder   (CH1/CH2 = PA8/PA9)
 *   TIM2  encoder sample tick  (100 Hz, no pins)
 *   TIM3  motor + LED PWM      (CH1/CH2/CH3 = PA6/PA7/PB0)
 *   TIM4  right wheel encoder  (CH1/CH2 = PB6/PB7)
 *
 * All four are spoken for, which is why the 1 ms tick runs on SysTick.
 */
#define CAR_TICK_IRQ_PRIORITY   1u

/* ===== CAN vehicle bus ====================================================
 * drv_can fixes the pins at PB8 (RX) / PB9 (TX) and the rate at 500 kbit/s.
 * Message layout is contracts/adas.dbc; do not hand-code identifiers.
 *
 * PIN NOTE: PB8/PB9 are TIM4 CH3/CH4. TIM4 drives the right encoder on CH1/CH2
 * (PB6/PB7) but not CH3/CH4, so there is no conflict - check here before
 * assigning anything new.
 */
#define CAR_NODE_ID_VEHICLE     2u        /* NodeAddress VC in adas.dbc       */

/* CAN_MODE_LOOPBACK drives the wire but needs no ACK and cannot bus-off - use
 * it when bringing this node up alone. CAN_MODE_NORMAL for real operation.
 *
 * CAUTION: in loopback this node never hears the gateway, so no command will
 * ever arrive and the motors stay stopped. That is safe, but it means loopback
 * cannot be used to test driving. */
#define CAR_CAN_MODE            CAN_MODE_NORMAL

/* Transmit periods are taken from the GENERATED header, not duplicated here:
 * they are a property of the contract, and a copy in each node's config is a
 * copy that can drift from the DBC. Change GenMsgCycleTime in adas.dbc and
 * re-run contracts/generate.sh. */
#define CAR_CAN_STATUS_PERIOD_MS ADAS_VC_STATUS_CYCLE_TIME_MS    /* 0x300, 0x310 */
#define CAR_CAN_HEARTBEAT_MS     ADAS_VC_HEARTBEAT_CYCLE_TIME_MS /* 0x701        */

/* Three missed cycles of 0x200. The car stops if driver intent goes quiet for
 * this long - independent of, and faster than, the gateway's RF failsafe. */
#define CAR_CAN_CMD_TIMEOUT_MS  (3u * ADAS_GATEWAY_DRIVER_CMD_CYCLE_TIME_MS)

/* ===== Status LED ========================================================= */
#define CAR_LED_STATUS_PORT     GPIOC
#define CAR_LED_STATUS_PIN      13u
#define CAR_LED_ACTIVE_LOW      1

/* ===== Indicator LEDs =====================================================
 * They show what was APPLIED, which only this node knows - the gateway sees
 * intent, not outcome. */
#define CAR_DRIVE_TIMER         TIM3
#define CAR_DRIVE_CHANNEL       PWM_CH3
#define CAR_DRIVE_PORT          GPIOB
#define CAR_DRIVE_PIN           0u

/* Lit while the REVERSE bit is set. Plain GPIO — PB5 has no peripheral role
 * in this design. */
#define CAR_REVERSE_LED_PORT    GPIOB
#define CAR_REVERSE_LED_PIN     5u

/* ===== TB6612FNG motor driver ============================================= */
/* One timer drives both motors: separate CCR per channel.
 *
 * 20 kHz is above the audible band, so the motors do not whine, and is well
 * inside the TB6612's 100 kHz input limit. */
#define CAR_MOTOR_PWM_TIMER     TIM3
#define CAR_MOTOR_PWM_HZ        20000u

/* Left motor: PWMA + AIN1/AIN2. */
#define CAR_MOTOR_L_PWM_CHANNEL PWM_CH1
#define CAR_MOTOR_L_PWM_PORT    GPIOA
#define CAR_MOTOR_L_PWM_PIN     6u
#define CAR_MOTOR_L_IN1_PORT    GPIOA
#define CAR_MOTOR_L_IN1_PIN     0u
#define CAR_MOTOR_L_IN2_PORT    GPIOA
#define CAR_MOTOR_L_IN2_PIN     1u

/* Right motor: PWMB + BIN1/BIN2. */
#define CAR_MOTOR_R_PWM_CHANNEL PWM_CH2
#define CAR_MOTOR_R_PWM_PORT    GPIOA
#define CAR_MOTOR_R_PWM_PIN     7u
#define CAR_MOTOR_R_IN1_PORT    GPIOA
#define CAR_MOTOR_R_IN1_PIN     2u
#define CAR_MOTOR_R_IN2_PORT    GPIOA
#define CAR_MOTOR_R_IN2_PIN     3u

/* STBY: low puts both H-bridges in standby (outputs off) regardless of the
 * direction pins. Held low until TB6612_Init() has configured everything. */
#define CAR_MOTOR_STBY_PORT     GPIOA
#define CAR_MOTOR_STBY_PIN      4u

/* Set to 1 if a wheel runs backwards when commanded forward */
#define CAR_MOTOR_L_INVERT      0
#define CAR_MOTOR_R_INVERT      0

/* ===== Wheel encoders =====================================================
 * Quadrature encoders on the timers' hardware encoder interfaces, so counting
 * costs no CPU time. Each timer's CH1/CH2 pins are fixed:
 * TIM1 = PA8/PA9, TIM4 = PB6/PB7. */
#define CAR_ENC_L_TIMER         TIM1
#define CAR_ENC_L_PORT_A        GPIOA
#define CAR_ENC_L_PIN_A         8u
#define CAR_ENC_L_PORT_B        GPIOA
#define CAR_ENC_L_PIN_B         9u
#define CAR_ENC_L_INVERT        false

#define CAR_ENC_R_TIMER         TIM4
#define CAR_ENC_R_PORT_A        GPIOB
#define CAR_ENC_R_PIN_A         6u
#define CAR_ENC_R_PORT_B        GPIOB
#define CAR_ENC_R_PIN_B         7u
#define CAR_ENC_R_INVERT        false

/* Gearmotor geometry. Counts per output-shaft revolution =
 * PPR x gearbox ratio x edge multiplier. ENCODER_MODE_BOTH counts both edges
 * of both phases, hence 4. */
#define CAR_ENC_PPR             11.0f
#define CAR_ENC_GEAR_RATIO      34.0f
#define CAR_ENC_EDGE_MULT       4.0f
#define CAR_ENC_COUNTS_PER_REV  (CAR_ENC_PPR * CAR_ENC_GEAR_RATIO * \
                                 CAR_ENC_EDGE_MULT)

/* Speed sampling. The rate is requested in hertz, so it stays correct at any
 * SYSCLK — unlike the prescaler/period pair it replaces, which was only 100 Hz
 * if the part happened to be running at 72 MHz. */
#define CAR_ENC_SAMPLE_TIMER    TIM2
#define CAR_ENC_SAMPLE_HZ       100u
#define CAR_ENC_SAMPLE_IRQ_PRIORITY  2u

/* ===== Vehicle geometry ===================================================
 * Needed to turn encoder RPM into the mm/s and mrad/s that VC_Motion carries.
 *
 * MEASURE THESE. They are plausible defaults for a small differential-drive
 * chassis, not measured values, and every speed the AEB reasons about scales
 * directly with the wheel diameter. */
#define CAR_WHEEL_DIAMETER_MM   65.0f     /* tyre outer diameter              */
#define CAR_TRACK_WIDTH_MM      150.0f    /* centre-to-centre, drive wheels   */

#endif /* VEHICLE_CONFIG_H_ */
