
#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif


#define SYSTEM_CLOCK_HZ            (72000000U)


#define UART_BAUDRATE              (115200U)


#define I2C_CLOCK_SPEED            (400000U)


#define CAN_BITRATE                (500000U)


#define TIMER_TICK_US              (1000U)
#define PWM_FREQUENCY              (20000U)


#define VL53_UPDATE_PERIOD_MS      (50U)
#define MPU_UPDATE_PERIOD_MS       (10U)


#define ENABLE_DEBUG               (1U)

#define ENABLE_UART                (1U)
#define ENABLE_CAN                 (1U)
#define ENABLE_I2C                 (1U)

#define ENABLE_VL53                (1U)
#define ENABLE_MPU                 (1U)

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */