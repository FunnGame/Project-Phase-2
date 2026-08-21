/*
 * nrf24_hal.h
 *
 * NRF24L01 Hardware Abstraction Layer
 * Communication ECU - Receiver (RX Mode)
 */

#ifndef PLATFORM_INC_NRF24_HAL_H_
#define PLATFORM_INC_NRF24_HAL_H_

#include "shared/nrf24/nrf24.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Lấy instance cấu hình SPI adapter cho nRF24.
 *         Instance này đóng vai trò cầu nối (bridge) giữa Generic Driver
 *         và SPI2 driver.
 *
 * @return const nrf24_hal_t* Con trỏ trỏ tới struct các hàm HAL hợp lệ.
 */
const nrf24_hal_t* NRF24_HAL_GetAdapter(void);

#ifdef __cplusplus
}
#endif

#endif /* PLATFORM_INC_NRF24_HAL_H_ */
