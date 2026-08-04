/**
 ******************************************************************************
 * @file    car_config.h
 * @brief   Central board configuration for the car's RF gateway node.
 *
 * Edit pins, ports, peripheral instances and RF-link parameters HERE — main.c
 * reads everything from these macros, so re-wiring never means hunting through
 * the code.
 *
 * Default target: STM32F103C8 ("Blue Pill").
 *
 * IMPORTANT — the RF section must match station/app/app_config.h exactly, or
 * the two radios will not hear each other.
 ******************************************************************************
 */
#ifndef CAR_CONFIG_H_
#define CAR_CONFIG_H_

#include "drv_common.h"
#include "drv_gpio.h"
#include "drv_spi.h"
#include "drv_pwm.h"
#include "drv_encoder.h"
#include "nrf24.h"
#include "rf_protocol.h"

/* ===== System clock ======================================================= */
/* HSE crystal frequency in Hz.*/
#define CAR_HSE_HZ              8000000u

/* ===== Timer budget =======================================================
 *   TIM1  left wheel encoder   (CH1/CH2 = PA8/PA9)
 *   TIM2  encoder sample tick  (100 Hz, no pins)
 *   TIM3  motor + LED PWM      (CH1/CH2/CH3 = PA6/PA7/PB0)
 *   TIM4  right wheel encoder  (CH1/CH2 = PB6/PB7)
 */
#define CAR_TICK_IRQ_PRIORITY   1u

/* ===== Status LED ========================================================= */
#define CAR_LED_STATUS_PORT     GPIOA
#define CAR_LED_STATUS_PIN      5u
#define CAR_LED_ACTIVE_LOW      0

/* ===== Indicator LEDs ===================================================== */
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

/* ===== nRF24 SPI bus ====================================================== */
/* SPI2 on the F103: PB13 SCK, PB14 MISO, PB15 MOSI. */
#define CAR_NRF_SPI             SPI2
#define CAR_NRF_SPI_PORT        GPIOB
#define CAR_NRF_SCK_PIN         13u
#define CAR_NRF_MISO_PIN        14u
#define CAR_NRF_MOSI_PIN        15u
#define CAR_NRF_SPI_BAUD        SPI_BAUD_DIV8   /* 36 MHz PCLK1 / 8 = 4.5 MHz */

/* ===== nRF24 control lines ================================================ */
#define CAR_NRF_CSN_PORT        GPIOB
#define CAR_NRF_CSN_PIN         12u
#define CAR_NRF_CE_PORT         GPIOB
#define CAR_NRF_CE_PIN          10u
#define CAR_NRF_IRQ_PORT        GPIOB
#define CAR_NRF_IRQ_PIN         11u

/* ===== RF link parameters — MUST MATCH THE STATION ======================== */
#define CAR_RF_CHANNEL          76u
#define CAR_RF_ADDRESS          { 0xE7u, 0xE7u, 0xE7u, 0xE7u, 0xE7u }
#define CAR_RF_PAYLOAD          RF_CONTROL_FRAME_SIZE   /* 7 bytes */
#define CAR_RF_DATARATE         NRF24_DR_1MBPS
#define CAR_RF_POWER            NRF24_PWR_0DBM
#define CAR_RF_AUTO_ACK         true

/* ===== Behaviour ========================================================== */
#define CAR_FAILSAFE_MS         150u

#endif /* CAR_CONFIG_H_ */
