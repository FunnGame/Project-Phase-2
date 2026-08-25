/**
 ******************************************************************************
 * @file    sensor_config.h
 * @brief   Central pin & sensor configuration for the VL53L0X sensor node.
 *
 * Edit pins, the I2C/CAN instances, the sensor addresses and the ranging
 * profile HERE — main.c and hal_vl53.c read everything from these macros, so
 * re-wiring never means hunting through the code.
 *
 * Default target: STM32F103C8 ("Blue Pill").
 ******************************************************************************
 */
#ifndef SENSOR_CONFIG_H_
#define SENSOR_CONFIG_H_

#include "drv_common.h"   /* CMSIS peripheral handles (GPIOB, I2C1), bool     */
#include "drv_can.h"      /* CAN_Mode, for SENSOR_CAN_MODE below              */

/* ===== System clock ======================================================= */
/* HSE crystal in Hz. The Blue Pill fits 8 MHz; 0 stays on the 8 MHz HSI. */
#define SENSOR_HSE_HZ           8000000u

/* ===== CAN vehicle bus ====================================================
 * Replaces the debug UART: ranges are published as CAN object lists per
 * contracts/adas.dbc, which a USB-CAN adapter decodes directly.
 *
 * drv_can fixes the pins at PB8 (RX) / PB9 (TX) and the rate at 500 kbit/s.
 * NOTE those are also I2C1's remap pins - SENSOR_I2C_REMAP must stay false or
 * the two peripherals fight over PB8/PB9. */
#define SENSOR_NODE_ID          3u        /* SF, per NodeAddress in adas.dbc  */

/* Bus mode. CAN_MODE_NORMAL needs another node to acknowledge every frame.
 *
 * If nothing on the bus is ACKing - adapter misconfigured, not connected, wrong
 * bit rate - the transmit error counter climbs by 8 per frame and this node is
 * bus-off in well under a second, looking exactly like broken wiring.
 *
 * Set CAN_MODE_LOOPBACK to take the far end out of the equation: the peripheral
 * still drives CANTX with real frames a sniffer can decode, but ignores CANRX,
 * so no ACK is required and bus-off is impossible. It cannot HEAR anything in
 * this mode, so it is for one-way bring-up only - put it back to NORMAL before
 * the vehicle-control node needs to talk back. */
#define SENSOR_CAN_MODE         CAN_MODE_NORMAL

/* Sensors range concurrently on this period; it must match
 * VL53_INTER_PERIOD_MS below. One raw slot per sensor fits inside it. */
#define SENSOR_MEAS_PERIOD_MS   50u
#define SENSOR_HEARTBEAT_MS     100u

/* A sensor that has produced nothing for this long is declared FAULT and drops
 * out of the fused object. Three measurement periods: long enough to ride out
 * one missed cycle, short enough that a dead sensor cannot keep contributing a
 * stale range to an AEB decision. THIS IS THE POINT OF IT - without the sweep,
 * a sensor whose I2C dies simply freezes at its last reading and goes on
 * looking like a valid target forever. */
#define SENSOR_STALE_MS         (3u * SENSOR_MEAS_PERIOD_MS)

/* ===== Status LED =========================================================
 * The only diagnostic left once the UART is gone, so init failures are blink
 * codes. PC13 is the Blue Pill's on-board LED - active low, no wiring. */
#define SENSOR_LED_PORT         GPIOC
#define SENSOR_LED_PIN          13u
#define SENSOR_LED_ACTIVE_LOW   1

/* Blink codes: N flashes, pause, repeat. Matches docs/architecture.md §5. */
#define SENSOR_BLINK_CLOCK      2u
#define SENSOR_BLINK_CAN        3u
#define SENSOR_BLINK_I2C        4u
#define SENSOR_BLINK_VL53       5u

/* ===== VL53L0X I2C bus ====================================================
 * MUST match VL53_I2C_BUS in vl53l0x_platform.c (which defaults to I2C1).
 * I2C1 pins are PB6 (SCL) / PB7 (SDA).
 *
 * All four sensors share this one bus. That is the whole reason the XSHUT
 * dance below exists. */
#define SENSOR_I2C              I2C1
#define SENSOR_I2C_HZ           400000u
#define SENSOR_I2C_REMAP        false     /* I2C1: false = PB6/PB7, true = PB8/PB9 */

/* ===== VL53L0X ranging profile (applied to every sensor) ================== */
#define VL53_DEFAULT_ADDR       0x29u     /* address every sensor boots at    */
#define VL53_BOOT_DELAY_MS      3u        /* Tboot ~1.2 ms + margin           */
#define VL53_TIMING_BUDGET_US   50000u    /* per-measurement budget           */
#define VL53_INTER_PERIOD_MS    50u       /* continuous-mode period           */

/* ===== Acceptance window ==================================================
 * A reading outside this band is treated as NO_TARGET no matter what number
 * the sensor returned. Two independent things make this necessary:
 *
 *   1. The ST API always fills in RangeMilliMeter, even when it has just
 *      decided the measurement failed. An empty scene comes back as ~8190 mm
 *      with RangeStatus = PHASE - a plausible-looking distance attached to a
 *      measurement the API itself does not believe.
 *   2. Even a VALID reading can exceed what the part is specified to do.
 *
 * MAX is the datasheet's ceiling for the DEFAULT ranging profile, which is
 * what vl53_configure() sets: no long-range mode, no relaxed signal-rate
 * limit. ST quotes ~2 m against a white target in low ambient light, and well
 * under that (~1.2 m) against a dark target or in sunlight. Anything claiming
 * more than 2 m from this profile is not a measurement.
 *
 * If you ever switch to long-range mode (signal-rate limit 0.1 MCPS, VCSEL
 * periods 18/14), raise this to 2500-3000 - but note SF_Range is declared
 * [0|4000] mm in adas.dbc, which is the real hard ceiling for the contract.
 *
 * MIN guards the other end: below the part's minimum the return can fold back
 * and read as a plausible mid-range distance. The API usually catches this as
 * RangeStatus = MINRANGE, but the floor is cheap and does not depend on it. */
#define VL53_MIN_VALID_MM       30u
#define VL53_MAX_VALID_MM       2000u

/* ===== Per-sensor wiring: the four-element front array ====================
 * Indices run LEFT to RIGHT across the front of the car, and that ordering is
 * the contract: it is what SFR_SensorIdx means on 0x101, so a trace can be
 * read as a spatial picture without a lookup table. Re-ordering these without
 * re-ordering the physical sensors silently corrupts every diagnostic.
 *
 * Every VL53L0X boots at VL53_DEFAULT_ADDR (0x29), so four on one bus all
 * answer at once. HAL_VL53_Init() resolves that by holding every XSHUT low,
 * then releasing ONE sensor at a time and readdressing it before waking the
 * next. Each XSHUT must therefore be its own free GPIO - they cannot be tied
 * together - and each address must be unique and not 0x29.
 *
 * PIN NOTE: PB6/PB7 are the I2C bus and PB8/PB9 are CAN, so the XSHUT lines
 * use PB0/PB1 and PB10/PB11. PB3/PB4 are deliberately avoided: they are JTAG
 * (JTDO/NJTRST) on the F103 and need a debug-port remap before they can be
 * used as plain GPIO.
 *
 * Addresses are 7-bit. */
#define VL53_FL_XSHUT_PORT      GPIOB     /* far left                         */
#define VL53_FL_XSHUT_PIN       0u
#define VL53_FL_ADDR            0x30u

#define VL53_CL_XSHUT_PORT      GPIOB     /* centre-left                      */
#define VL53_CL_XSHUT_PIN       1u
#define VL53_CL_ADDR            0x31u

#define VL53_CR_XSHUT_PORT      GPIOB     /* centre-right                     */
#define VL53_CR_XSHUT_PIN       10u
#define VL53_CR_ADDR            0x32u

#define VL53_FR_XSHUT_PORT      GPIOB     /* far right                        */
#define VL53_FR_XSHUT_PIN       11u
#define VL53_FR_ADDR            0x33u

#endif /* SENSOR_CONFIG_H_ */
