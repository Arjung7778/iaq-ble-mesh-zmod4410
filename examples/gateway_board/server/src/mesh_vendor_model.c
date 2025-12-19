#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "access.h"
#include "access_config.h"
#include "access_reliable.h"
#include "nrf_mesh_defines.h"
#include "nrf_mesh.h"
#include "log.h"
#include "app_uart_gateway.h"

#include "mesh_vendor_model.h"
#include "device_state_manager.h"
#include "nrf_strerror.h"

#define VENDOR_COMPANY_ID   0x0059
#define VENDOR_MODEL_ID     0x1234
#define VENDOR_OPCODE_SENSOR_VALUES  0xC1
#define VENDOR_PAYLOAD_MAX  8

/* Default group address for publishing - configure this or use the one set via app */
#define DEFAULT_PUBLISH_ADDRESS  0xC000

static bool s_vendor_model_ready = false;
static bool s_publish_configured = false;
static uint16_t s_publish_address = DEFAULT_PUBLISH_ADDRESS;
static dsm_handle_t s_appkey_handle = DSM_HANDLE_INVALID;
static dsm_handle_t s_publish_addr_handle = DSM_HANDLE_INVALID;




// Track first reception from each node for UART forwarding
#define MAX_TRACKED_NODES 10
static uint16_t s_received_nodes[MAX_TRACKED_NODES];
static uint8_t s_received_node_count = 0;

static bool is_first_reception_from_node(uint16_t src_addr)
{
    // Check if we've seen this node before
    for (uint8_t i = 0; i < s_received_node_count; i++)
    {
        if (s_received_nodes[i] == src_addr)
        {
            return false;  // Already received from this node
        }
    }
    
    // First time seeing this node - add to list
    if (s_received_node_count < MAX_TRACKED_NODES)
    {
        s_received_nodes[s_received_node_count] = src_addr;
        s_received_node_count++;
    }
    
    return true;  // First reception from this node
}




static access_model_handle_t m_vendor_model_handle = ACCESS_HANDLE_INVALID;

static void vendor_model_rx_cb(access_model_handle_t handle,
                               const access_message_rx_t * p_message,
                               void * p_args);

static const access_opcode_handler_t m_vendor_opcode_handlers[] =
{
    {
        .opcode = { VENDOR_OPCODE_SENSOR_VALUES, VENDOR_COMPANY_ID },
        .handler = vendor_model_rx_cb
    }
};

uint32_t mesh_vendor_model_init(void)
{
    access_model_add_params_t add_params;
    memset(&add_params, 0, sizeof(add_params));
    add_params.model_id.model_id = VENDOR_MODEL_ID;
    add_params.model_id.company_id = VENDOR_COMPANY_ID;
    add_params.element_index = 0;
    add_params.p_opcode_handlers = m_vendor_opcode_handlers;
    add_params.opcode_count = ARRAY_SIZE(m_vendor_opcode_handlers);
    add_params.p_args = NULL;
    add_params.publish_timeout_cb = NULL;

    uint32_t status = access_model_add(&add_params, &m_vendor_model_handle);
    if (status != NRF_SUCCESS)
    {
        m_vendor_model_handle = ACCESS_HANDLE_INVALID;
        s_vendor_model_ready = false;
        return status;
    }

    s_vendor_model_ready = true;
    s_publish_configured = false;
    
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Vendor model added (company=0x%04X, model=0x%04X), handle=%u\n",
          VENDOR_COMPANY_ID, VENDOR_MODEL_ID, (unsigned)m_vendor_model_handle);

    // Allocate subscription list
    status = access_model_subscription_list_alloc(m_vendor_model_handle);
    
    if (status == NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "Subscription list allocated successfully\n");
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
              "Failed to allocate subscription list: 0x%08X\n", status);
        return status;
    }

          
    return NRF_SUCCESS;
}

bool mesh_vendor_model_is_ready(void)
{
    return s_vendor_model_ready;
}

void mesh_vendor_model_publication_set(void)
{
    s_publish_configured = true;
    
    uint32_t status;
    
    // Get the configured publish address handle
    status = access_model_publish_address_get(m_vendor_model_handle, &s_publish_addr_handle);
    if (status == NRF_SUCCESS && s_publish_addr_handle != DSM_HANDLE_INVALID)
    {
        nrf_mesh_address_t addr;
        status = dsm_address_get(s_publish_addr_handle, &addr);
        if (status == NRF_SUCCESS)
        {
            s_publish_address = addr.value;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publish address: 0x%04X\n", s_publish_address);
        }
    }
    
    // Get the first bound AppKey
    dsm_handle_t appkey_handles[DSM_APP_MAX];
    uint16_t appkey_count = DSM_APP_MAX;
    
    status = access_model_applications_get(m_vendor_model_handle, appkey_handles, &appkey_count);
    if (status == NRF_SUCCESS && appkey_count > 0)
    {
        s_appkey_handle = appkey_handles[0];
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Using bound AppKey handle: %u\n", s_appkey_handle);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "No AppKey bound to model\n");
        s_appkey_handle = DSM_HANDLE_INVALID;
    }
    
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication configured for vendor model\n");
}

// Add this helper function at the top
static const char* get_iaq_description(uint8_t level)
{
    switch(level)
    {
        case 1: return "Very Good";
        case 2: return "Good";
        case 3: return "Medium";
        case 4: return "Poor";
        case 5: return "Bad";
        default: return "Unknown";
    }
}

// MODIFY your existing vendor_model_rx_cb to actually process messages:
static void vendor_model_rx_cb(access_model_handle_t handle,
                               const access_message_rx_t * p_message,
                               void * p_args)
{
    (void)handle;
    (void)p_args;

    if (p_message->opcode.opcode != VENDOR_OPCODE_SENSOR_VALUES ||
        p_message->opcode.company_id != VENDOR_COMPANY_ID)
    {
        return;
    }

    // Extract source address
    uint16_t src_addr = p_message->meta_data.src.value;
    
    // Get own address to filter out own messages
    dsm_local_unicast_address_t local_addr;
    dsm_local_unicast_addresses_get(&local_addr);
    
    if (src_addr == local_addr.address_start)
    {
        return;  // Don't display own published data
    }

    // Check if this is first reception from this node
    bool is_first = is_first_reception_from_node(src_addr);
    
    if (is_first)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "*** FIRST DATA FROM NEW NODE 0x%04X ***\n", src_addr);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "*** SENSOR DATA FROM NODE 0x%04X ***\n", src_addr);
    }

    if (p_message->length >= 6)
    {
        const uint8_t *data = p_message->p_data;

        uint8_t iaq_level = data[0];
        uint16_t tvoc_x100 = (uint16_t)(data[1] | (data[2] << 8));
        uint16_t eco2 = (uint16_t)(data[3] | (data[4] << 8));
        uint8_t iaq_x10 = data[5];

        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "Node 0x%04X: IAQ=%u.%u (%s) | TVOC=%u.%02u mg/m3 | eCO2=%u ppm\n",
              src_addr,
              iaq_x10 / 10, iaq_x10 % 10,
              get_iaq_description(iaq_level), 
              tvoc_x100 / 100, tvoc_x100 % 100,
              eco2);
       
        // Send ALL received data to UART (first and subsequent)
        app_uart_send_iaq_data(src_addr, (float)iaq_x10 / 10.0f, (float)tvoc_x100 / 100.0f, (float)eco2);
        
        if (is_first)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "→ Sent to UART (FIRST READING from this node)\n");
        }
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN,
              "Node 0x%04X: Invalid message length: %u\n", 
              src_addr, p_message->length);
    }
}


static void pack_payload(uint8_t iaq_level, float iaq_float, uint16_t tvoc_x100, uint16_t eco2,
                         uint8_t * buf, uint8_t * out_len)
{
    buf[0] = iaq_level;                           // IAQ Level: 1-5
    buf[1] = (uint8_t)(tvoc_x100 & 0xFF);         // TVOC low byte
    buf[2] = (uint8_t)((tvoc_x100 >> 8) & 0xFF);  // TVOC high byte
    buf[3] = (uint8_t)(eco2 & 0xFF);              // eCO2 low byte
    buf[4] = (uint8_t)((eco2 >> 8) & 0xFF);       // eCO2 high byte
    
    // Store IAQ float × 10 (e.g., 1.2 → 12, 4.5 → 45)
    uint8_t iaq_x10 = (uint8_t)(iaq_float * 10.0f + 0.5f);
    buf[5] = iaq_x10;

    *out_len = 6;
}


void mesh_publish_sensor_values(float iaq, float tvoc, float eco2)
{
    static uint32_t s_warn_count = 0;
    
    if (m_vendor_model_handle == ACCESS_HANDLE_INVALID)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Vendor model not initialized\n");
        return;
    }

    // NaN check
    if (iaq != iaq || tvoc != tvoc || eco2 != eco2)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "NaN values detected\n");
        return;
    }

    if (!s_publish_configured)
    {
        s_warn_count++;
        if (s_warn_count % 10 == 1)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_WARN,
                  "Publish not configured (attempt %u)\n", s_warn_count);
        }
        return;
    }
    
    if (s_appkey_handle == DSM_HANDLE_INVALID)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "AppKey handle invalid\n");
        return;
    }

    // Clamp and convert IAQ to 1-5 rating
    uint8_t iaq_level;
    if (iaq < 2.0f) 
        iaq_level = 1;      // Level 1: Very Good
    else if (iaq < 3.0f) 
        iaq_level = 2;      // Level 2: Good
    else if (iaq < 4.0f) 
        iaq_level = 3;      // Level 3: Medium
    else if (iaq < 5.0f) 
        iaq_level = 4;      // Level 4: Poor
    else 
        iaq_level = 5;      // Level 5: Bad
    
    // Clamp TVOC
    if (tvoc < 0.0f) tvoc = 0.0f;
    if (tvoc > 655.35f) tvoc = 655.35f;
    
    // Clamp eCO2
    if (eco2 < 0.0f) eco2 = 0.0f;
    if (eco2 > 65535.0f) eco2 = 65535.0f;

    uint8_t payload[VENDOR_PAYLOAD_MAX];
    uint8_t payload_len;

    uint16_t tvoc_x100 = (uint16_t)(tvoc * 100.0f + 0.5f);  // Round to nearest
    uint16_t eco2_i = (uint16_t)(eco2 + 0.5f);

    pack_payload(iaq_level, iaq, tvoc_x100, eco2_i, payload, &payload_len);

    access_message_tx_t tx;
    memset(&tx, 0, sizeof(tx));

    tx.opcode.opcode = VENDOR_OPCODE_SENSOR_VALUES;
    tx.opcode.company_id = VENDOR_COMPANY_ID;
    tx.p_buffer = payload;
    tx.length = payload_len;
    tx.force_segmented = false;
    tx.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    tx.access_token = nrf_mesh_unique_token_get();

    uint32_t status = access_model_publish(m_vendor_model_handle, &tx);

    if (status == NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "Published: IAQ_Level=%u, TVOC_x100=%u, eCO2=%u\n",
              iaq_level, tvoc_x100, eco2_i);
    }
    else
    {
        s_publish_configured = false;
        const char *err_str = nrf_strerror_get(status);
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
              "Publish failed: 0x%08X (%s)\n", status, (err_str ? err_str : "unknown"));
    }
}

access_model_handle_t mesh_vendor_model_handle_get(void)
{
    return m_vendor_model_handle;
}
