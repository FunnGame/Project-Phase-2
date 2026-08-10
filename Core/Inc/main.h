/*
 * main.h
 * Author: trong
 */
#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx.h"
#include <stdint.h>
#include <stdbool.h>

/* Có thể thêm định nghĩa các chân LED báo trạng thái ở đây nếu có */

void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
