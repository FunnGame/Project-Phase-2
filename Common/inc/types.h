
#ifndef TYPEDEF_H
#define TYPEDEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef uint8_t     uint8;
typedef uint16_t    uint16;
typedef uint32_t    uint32;
typedef uint64_t    uint64;

typedef int8_t      sint8;
typedef int16_t     sint16;
typedef int32_t     sint32;
typedef int64_t     sint64;


typedef float       float32;
typedef double      float64;


typedef enum
{
    FALSE = 0,
    TRUE  = 1
} Bool_t;


typedef enum
{
    STATUS_OK = 0,

    STATUS_ERROR,

    STATUS_BUSY,

    STATUS_TIMEOUT,

    STATUS_INVALID_PARAM,

    STATUS_NOT_INITIALIZED

} Status_t;


#ifndef NULL
#define NULL ((void *)0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* TYPEDEF_H */