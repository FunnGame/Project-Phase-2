#include "stm32f1xx.h"
#include "gateway.h"
#include "drv_can.h"
#include "shared/nrf24/nrf24.h"
#include "contracts/rf_protocol.h"

static rf_control_parser_t rf_parser;
static nrf24_t             nrf_device;

void SysTick_Handler(void) {
    Gateway_Tick1ms();
}

int main(void) {
    /* 1. System Tick Init 1ms */
    SysTick_Config(SystemCoreClock / 1000U);

    /* 2. Hardware & Gateway Layer Init */
    Gateway_Init();
    rf_control_parser_reset(&rf_parser);

    /* 3. Main Operational Loop */
    while (1) {
        /* A. Luồng xử lý RF Receiver */[cite: 9]
        if (nrf24_data_available(&nrf_device)) {
            uint8_t raw_byte;
            if (nrf24_read(&nrf_device, &raw_byte, 1U)) {
                rf_control_frame_t decoded_frame;
                /* Giải mã dòng byte streaming qua Parser */[cite: 9]
                if (rf_control_parser_feed(&rf_parser, raw_byte, &decoded_frame)) {
                    Gateway_ProcessRFFrame(&decoded_frame);
                }
            }
        }

        /* B. Luồng xử lý nhận tin nhắn CAN Telemetry */
        CAN_RxHeader_t rx_header;
        if (DRV_CAN_Receive(&rx_header)) {
            if (rx_header.std_id == CAN_ID_TELEMETRY_STATUS) {
                CAN_TelemetryStatus_t telemetry;
                CANMatrix_UnpackTelemetry(rx_header.data, &telemetry);
                /* Sẵn sàng cho luồng đẩy Telemetry ngược về HMI trạm */
            }
        }
    }
}
