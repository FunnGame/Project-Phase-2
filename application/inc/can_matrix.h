#ifndef CAN_MATRIX_H
#define CAN_MATRIX_H

#include <stdint.h>

#define CAN_ID_CONTROL_CMD        0x100
#define CAN_ID_SENSOR_DISTANCE    0x200
#define CAN_ID_TELEMETRY_STATUS   0x300
#define CAN_ID_SYSTEM_POST        0x400

typedef enum
{
    MODE_MANUAL = 0,
    MODE_AUTO   = 1
} SystemMode_t;

typedef struct
{
    int8_t throttle;
    int8_t steering;
    uint8_t mode;
    uint8_t flags;
    uint8_t seq;
    uint8_t crc;
} CAN_ControlCmd_t;

typedef struct
{
    uint16_t distance_mm;
} CAN_SensorDistance_t;

typedef struct
{
    uint16_t speed_rpm;
    uint8_t acc_active;
    uint8_t aeb_active;
    uint8_t battery_percent;
} CAN_TelemetryStatus_t;

typedef struct
{
    uint8_t rf_ok;
    uint8_t can_ok;
    uint8_t spi_ok;
} CAN_PostStatus_t;

#endif
