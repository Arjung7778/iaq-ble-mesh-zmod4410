#ifndef APP_UART_GATEWAY_H__
#define APP_UART_GATEWAY_H__

#include <stdint.h>

/**
 * @brief Initialize UART for ESP32-S3 communication
 * 
 * Configures UART at 115200 baud, 8N1, no flow control
 */
void app_uart_gateway_init(void);
/* 
 * Sends JSON format: {"node":"0x0029","iaq":2.3,"tvoc":0.45,"eco2":680}\n
 */
void app_uart_send_iaq_data(uint16_t node_addr, float iaq, float tvoc, float eco2);

#endif /* APP_UART_GATEWAY_H__ */