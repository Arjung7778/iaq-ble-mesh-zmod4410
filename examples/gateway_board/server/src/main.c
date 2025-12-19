#include <stdint.h>
#include <string.h>

#include "boards.h"
#include "simple_hal.h"
#include "app_timer.h"
#include "app_scheduler.h"

#include "nrf_mesh_assert.h"
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "proxy.h"
#include "nrf_power.h"
#include "mesh_config_entry.h"
#include "mesh_config.h"

#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

#include "log.h"
#include "rtt_input.h"

#include "app_config.h"
#include "example_common.h"
#include "nrf_mesh_config_examples.h"
#include "ble_softdevice_support.h"

/* IAQ + vendor model */
#include "mesh_vendor_model.h"
#include "app_sensor_iaq.h"
#include "mesh_vendor_client.h"

#include "app_uart_gateway.h"

#define SCHED_QUEUE_SIZE       32
#define SCHED_EVENT_DATA_SIZE  16


static bool m_device_provisioned;

/* Forward declarations for callbacks */
static void config_server_evt_cb(const config_server_evt_t * p_evt);
static void provisioning_device_identification_start_cb(uint8_t attention_duration_sec);
static void provisioning_device_identification_stop_cb(void);
static void provisioning_complete_cb(void);
static void provisioning_abort_cb(void);
static void models_init_cb(void);


void mesh_assertion_handler(uint32_t pc)
{
    //__LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Mesh assertion at PC: 0x%08X\n", pc);
    return;
}

/* Utility: print unicast address */
static void unicast_address_print(void)
{
    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x", node_address.address_start);
}

/* Node reset helper */
static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset -----");
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
    else if (p_evt->type == CONFIG_SERVER_EVT_MODEL_PUBLICATION_SET)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication set event received\n");
        
        // Check if it's for our vendor model (element 0, vendor model)
        access_model_handle_t handle = mesh_vendor_model_handle_get();
        if (handle != ACCESS_HANDLE_INVALID)
        {
            // Mark publication as configured
            mesh_vendor_model_publication_set();
        }
    }
    else if (p_evt->type == CONFIG_SERVER_EVT_APPKEY_ADD)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "AppKey added\n");
    }
    else if (p_evt->type == CONFIG_SERVER_EVT_MODEL_APP_BIND)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Model AppKey bound\n");
    }
}

/* Provisioning identification callbacks */
static void provisioning_device_identification_start_cb(uint8_t attention_duration_sec)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Device identification started");
    hal_led_mask_set(HAL_LED_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(HAL_LED_MASK_HALF,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_sec));
}

static void provisioning_device_identification_stop_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Device identification stopped");
    hal_led_blink_stop();
}


static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned");

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    unicast_address_print();
    hal_led_blink_stop();
    hal_led_mask_set(HAL_LED_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);

    /* DO NOT attempt to add a model after provisioning.
       Only start IAQ if the vendor model was already added at startup. */
    if (mesh_vendor_model_is_ready())
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Vendor model present — starting IAQ.");
        app_sensor_iaq_start();
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Vendor model not present — IAQ not started");
        /* Optional: start IAQ in degraded mode (without publishing) or take other action. */
    }
}

static void provisioning_abort_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Provisioning aborted");
    hal_led_blink_stop();
}

/* Initialize models: register the vendor model only and any other app models */
static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing models...");

    /* Initialize vendor model ONLY (handles both publish and subscribe) */
    mesh_vendor_model_init();
    
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Models initialized");
}

/* Initialize mesh stack */
static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };

    uint32_t status = mesh_stack_init(&init_params, &m_device_provisioned);
    switch (status)
    {
        case NRF_ERROR_INVALID_DATA:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Persistent data corrupted. Starting unprovisioned.");
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Reboot device before starting provisioning.");
            break;
        case NRF_SUCCESS:
            break;
        default:
            APP_ERROR_CHECK(status);
    }
}

/* RTT input handler (simple usage) */
static const char m_usage_string[] =
    "\n"
    "\t\t---- IAQ Sensor Server ----\n"
    "\t\t Use nRF Mesh app to provision and configure publish/subscribe.\n"
    "\t\t---------------------------\n";

static void rtt_input_handler(int key)
{
    (void)key;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
}

/* initialize(): sets up logging, timers, BLE stack, mesh stack and IAQ subsystem (init only) */
static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- IAQ BLE Mesh Server (clean) -----");

    /* ----- IMPORTANT: initialize scheduler BEFORE creating/starting timers ----- */
    APP_SCHED_INIT(SCHED_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

    ERROR_CHECK(app_timer_init());
    hal_leds_init();
    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();

    /* Initialize IAQ subsystem (TWI, ZMOD init, IAQ algorithm). Does NOT start timers. */
    app_sensor_iaq_init();

    app_uart_gateway_init();
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "UART gateway initialized\n");
}

/* start(): handle provisioning or start mesh if provisioned, then start IAQ sampling timer */
static void start(void)
{
    rtt_input_enable(rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = provisioning_device_identification_start_cb,
            .prov_device_identification_stop_cb = provisioning_device_identification_stop_cb,
            .prov_abort_cb = provisioning_abort_cb,
            .p_device_uri = EX_URI_SENSOR_SERVER
        };

        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    else
    {
        unicast_address_print();
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

    /* --- START IAQ timer only if vendor model was added successfully --- */
    if (mesh_vendor_model_is_ready())
    {
        app_sensor_iaq_start();
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Vendor model not ready — IAQ timer not started");
        /* Option: implement a retry/attempt to add model later */
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);

    hal_led_mask_set(HAL_LED_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

/* main */
int main(void)
{

    nrf_power_dcdcen_set(1);

    initialize(); 
    start();

    for (;;)
    {
        app_sched_execute();
        (void)sd_app_evt_wait();
    }
}