#include "app_uart_gateway.h"
#include "app_uart.h"
#include "app_fifo.h"
#include "nrf_uart.h"
#include "boards.h"
#include "log.h"
#include <string.h>

// FIFO buffer sizes
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 256

// Pin configuration
#define UART_TX_PIN  6
#define UART_RX_PIN  8

// FIFO buffers (allocated by APP_UART_FIFO_INIT macro)
static uint8_t m_rx_buf[UART_RX_BUF_SIZE];
static uint8_t m_tx_buf[UART_TX_BUF_SIZE];
static bool m_uart_initialized = false;

static void uart_event_handle(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            // Optional: handle RX data from ESP32
            break;

        case APP_UART_COMMUNICATION_ERROR:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, 
                  "UART error: 0x%X\n", p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, 
                  "FIFO error: 0x%X\n", p_event->data.error_code);
            break;

        case APP_UART_TX_EMPTY:
            // TX complete - buffer empty
            break;

        default:
            break;
    }
}

void app_uart_gateway_init(void)
{
    if (m_uart_initialized)
    {
        return;
    }

    uint32_t err_code;
    
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = UART_RX_PIN,
        .tx_pin_no    = UART_TX_PIN,
        .rts_pin_no   = UART_PIN_DISCONNECTED,
        .cts_pin_no   = UART_PIN_DISCONNECTED,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    app_uart_buffers_t buffers =
    {
        .rx_buf      = m_rx_buf,
        .rx_buf_size = sizeof(m_rx_buf),
        .tx_buf      = m_tx_buf,
        .tx_buf_size = sizeof(m_tx_buf)
    };

    err_code = app_uart_init(&comm_params, &buffers, uart_event_handle, APP_IRQ_PRIORITY_LOWEST);
    
    if (err_code != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, 
              "UART init failed: 0x%X\n", err_code);
        return;
    }
    
    m_uart_initialized = true;
    
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, 
          "UART FIFO initialized (TX=%d, RX=%d bytes)\n", 
          UART_TX_BUF_SIZE, UART_RX_BUF_SIZE);
    
    // Send startup message
    const char *msg = "{\"status\":\"nRF52 Ready\"}\n";
    for (int i = 0; msg[i]; i++)
    {
        app_uart_put(msg[i]);
    }
}

void app_uart_send_iaq_data(uint16_t node_addr, float iaq, float tvoc, float eco2)
{
    if (!m_uart_initialized)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "UART not initialized!\n");
        return;
    }
    
    if (iaq != iaq || tvoc != tvoc || eco2 != eco2)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "NaN detected in UART data!\n");
        return;
    }

    char buf[96];
    int len = snprintf(buf, sizeof(buf),
                       "{\"node\":\"0x%04X\",\"iaq\":%d.%d,\"tvoc\":%d.%02d,\"eco2\":%d}\n",
                       node_addr,
                       (int)iaq, (int)((iaq - (int)iaq) * 10),
                       (int)tvoc, (int)((tvoc - (int)tvoc) * 100),
                       (int)(eco2 + 0.5f));
    
    if (len > 0 && len < sizeof(buf))
    {

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending UART: %s", buf);
        
        for (int i = 0; i < len; i++)
        {
            app_uart_put(buf[i]);  
        }
        
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "UART sent successfully\n");
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "UART buffer overflow: len=%d\n", len);
    }
}
