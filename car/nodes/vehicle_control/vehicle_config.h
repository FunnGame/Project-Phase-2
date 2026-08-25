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
#include "drv_i2c.h"
#include "mpu6050.h"   /* MPU6050_ADDR_*, MPU6050_*_FS_* */
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
 * Message layout is contracts/adas.dbc; do not hand-code identifiers. */
#define CAR_NODE_ID_VEHICLE     2u        /* NodeAddress VC in adas.dbc       */

#define CAR_CAN_MODE            CAN_MODE_NORMAL

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

#define CAR_MOTOR_STBY_PORT     GPIOA
#define CAR_MOTOR_STBY_PIN      4u


#define CAR_MOTOR_L_INVERT      0
#define CAR_MOTOR_R_INVERT      0

/* ===== Wheel encoders =====================================================
 * TIM1 = PA8/PA9, TIM4 = PB6/PB7. */
#define CAR_ENC_L_TIMER         TIM1
#define CAR_ENC_L_PORT_A        GPIOA
#define CAR_ENC_L_PIN_A         8u
#define CAR_ENC_L_PORT_B        GPIOA
#define CAR_ENC_L_PIN_B         9u

#define CAR_ENC_L_INVERT        true

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

#define CAR_ENC_SAMPLE_TIMER    TIM2
#define CAR_ENC_SAMPLE_HZ       100u
#define CAR_ENC_SAMPLE_IRQ_PRIORITY  2u

/* ===== MPU6050 IMU ========================================================
 * SET TO 0 TO REMOVE THE IMU ENTIRELY.
 */
#define CAR_IMU_ENABLED         0

/* ===== MPU6050 wiring =====================================================
 * I2C2 (PB10 SCL / PB11 SDA)
 */
#define CAR_MPU_I2C             I2C2
#define CAR_MPU_I2C_HZ          100000u
#define CAR_MPU_ADDR            MPU6050_ADDR_AD0_LOW
#define CAR_MPU_ACCEL_RANGE     MPU6050_ACCEL_FS_4G
#define CAR_MPU_GYRO_RANGE      MPU6050_GYRO_FS_500DPS

/* Anti-alias filter and output rate. */
#define CAR_MPU_DLPF            MPU6050_DLPF_44HZ
#define CAR_MPU_SMPLRT_DIV      9u

/* Set to 1 if the board is mounted with its Z axis pointing DOWN,
*/
#define CAR_MPU_YAW_INVERT      0
#define CAR_MPU_BIAS_SAMPLES    64u

/* ===== Vehicle geometry ===================================================*/
#define CAR_WHEEL_DIAMETER_MM   65.0f     /* tyre outer diameter              */
#define CAR_TRACK_WIDTH_MM      200.0f    /* centre-to-centre, drive wheels   */

/* ===== Steering feel =======================================================
 *   GAIN  scales how much steering authority the stick has at all. Lower =
 *         wider turns. This is the one to reach for first.
 *   SLEW  limits how fast the applied steering may CHANGE, in percent per
 *         second.
*/
#define CAR_STEER_GAIN_PCT      60u       /* 100 = the raw arcade mix         */
#define CAR_STEER_SLEW_PCT_PER_S 250u     /* full lock in ~0.4 s              */


#endif /* VEHICLE_CONFIG_H_ */
