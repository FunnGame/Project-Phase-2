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
 * I2C1 pins are PB6 (SCL) / PB7 (SDA). */
#define SENSOR_I2C              I2C1
#define SENSOR_I2C_HZ           400000u
#define SENSOR_I2C_REMAP        false     /* I2C1: false = PB6/PB7, true = PB8/PB9 */

/* ===== VL53L0X ranging profile (applied to every sensor) ================== */
#define VL53_DEFAULT_ADDR       0x29u     /* address every sensor boots at    */
#define VL53_BOOT_DELAY_MS      3u        /* Tboot ~1.2 ms + margin           */
#define VL53_TIMING_BUDGET_US   50000u    /* per-measurement budget           */
#define VL53_INTER_PERIOD_MS    50u       /* continuous-mode period           */

/* ===== Per-sensor wiring ==================================================
 * Every VL53L0X boots at VL53_DEFAULT_ADDR; the XSHUT lines let the HAL bring
 * them up one at a time and give each a unique address. Each XSHUT must be a
 * free GPIO (not the I2C pins) and each address unique and != 0x29. */
#define VL53_FRONT_XSHUT_PORT   GPIOB
#define VL53_FRONT_XSHUT_PIN    0u
#define VL53_FRONT_ADDR         0x30u

/* Second sensor — kept here but currently disabled in hal_vl53.h for
 * single-sensor debug. Re-enable VL53_SIDE in the enum + table to use it. */
#define VL53_SIDE_XSHUT_PORT    GPIOB
#define VL53_SIDE_XSHUT_PIN     1u
#define VL53_SIDE_ADDR          0x31u

#endif /* SENSOR_CONFIG_H_ */
