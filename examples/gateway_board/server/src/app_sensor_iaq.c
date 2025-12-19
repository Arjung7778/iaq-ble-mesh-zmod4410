#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "app_sensor_iaq.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "nrf_drv_twi.h"
#include "nrf_error.h"
#include "nrf_assert.h"
#include "nrf_delay.h"

#include "log.h"
#include "mesh_vendor_model.h"

#include "zmod4xxx.h"
#include "zmod4410_config_iaq2.h"
#include "iaq_2nd_gen.h"

#ifndef APP_SENSOR_IAQ_MEAS_INTERVAL_MS
#define APP_SENSOR_IAQ_MEAS_INTERVAL_MS 1000
#endif

#ifndef TWI_SCL_PIN
#define TWI_SCL_PIN 27
#endif
#ifndef TWI_SDA_PIN
#define TWI_SDA_PIN 26
#endif

#define ZMOD4410_I2C_ADDR 0x32
#define TWI_INSTANCE_ID 0

#ifndef IAQ_2ND_GEN_OK
#define IAQ_2ND_GEN_OK 0
#endif
#ifndef IAQ_2ND_GEN_STABILIZATION
#define IAQ_2ND_GEN_STABILIZATION 1
#endif

#define IAQ_THRESHOLD       0.5f
#define TVOC_THRESHOLD      0.05f
#define ECO2_THRESHOLD      10.0f 


static uint16_t m_sample_count = 0;
static bool m_algorithm_stable = false;

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
APP_TIMER_DEF(m_iaq_timer_id);

static zmod4xxx_dev_t m_zmod_dev;
static uint8_t m_zmod_prod_data[ZMOD4410_PROD_DATA_LEN];
static uint8_t m_zmod_adc_result[ZMOD4410_ADC_DATA_LEN];
static iaq_2nd_gen_handle_t m_iaq_handle;
static iaq_2nd_gen_results_t m_iaq_results;
static iaq_2nd_gen_inputs_t m_iaq_inputs;
static bool m_sensor_initialized = false;
static bool m_timer_running = false;

typedef struct {
    float last_iaq;
    float last_tvoc;
    float last_eco2;
    bool first_reading;
} sensor_thresholds_t;

static sensor_thresholds_t m_thresholds = {
    .last_iaq = 0.0f,
    .last_tvoc = 0.0f,
    .last_eco2 = 0.0f,
    .first_reading = true
};

static void meas_timer_handler(void * p_context);
static void scheduled_meas_handler(void * p_event_data, uint16_t event_size);
static bool should_publish_data(float iaq, float tvoc, float eco2);

static bool is_valid_float(float val)
{
    if (val != val)
    {
        return false;
    }
    if (val > 1e10f || val < -1e10f)
    {
        return false;
    }
    return true;
}

static bool should_publish_data(float iaq, float tvoc, float eco2)
{
    if (m_thresholds.first_reading) {
        m_thresholds.first_reading = false;
        m_thresholds.last_iaq = iaq;
        m_thresholds.last_tvoc = tvoc;
        m_thresholds.last_eco2 = eco2;
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "First reading - publishing to MQTT\n");
        return true;
    }
    
    bool iaq_changed = fabsf(iaq - m_thresholds.last_iaq) >= IAQ_THRESHOLD;
    bool tvoc_changed = fabsf(tvoc - m_thresholds.last_tvoc) >= TVOC_THRESHOLD;
    bool eco2_changed = fabsf(eco2 - m_thresholds.last_eco2) >= ECO2_THRESHOLD;
    
    if (iaq_changed || tvoc_changed || eco2_changed) {
        m_thresholds.last_iaq = iaq;
        m_thresholds.last_tvoc = tvoc;
        m_thresholds.last_eco2 = eco2;
        
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, 
              "Threshold exceeded - IAQ: %s, TVOC: %s, eCO2: %s\n",
              iaq_changed ? "YES" : "NO",
              tvoc_changed ? "YES" : "NO",
              eco2_changed ? "YES" : "NO");
        return true;
    }
    
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "No significant change - skipping publish\n");
    return false;
}

static int8_t hal_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    ret_code_t err;
    
    err = nrf_drv_twi_tx(&m_twi, dev_addr, &reg_addr, 1, true);
    if (err != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "I2C read tx failed: 0x%x\n", err);
        return -1;
    }
    
    err = nrf_drv_twi_rx(&m_twi, dev_addr, data, len);
    if (err != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "I2C read rx failed: 0x%x\n", err);
        return -1;
    }
    
    return 0;
}

static int8_t hal_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    ret_code_t err;
    uint8_t buf[64];
    
    if (len + 1 > sizeof(buf))
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "I2C write buffer too small\n");
        return -1;
    }
    
    buf[0] = reg_addr;
    memcpy(&buf[1], data, len);
    
    err = nrf_drv_twi_tx(&m_twi, dev_addr, buf, len + 1, false);
    if (err != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "I2C write failed: 0x%x\n", err);
        return -1;
    }
    
    return 0;
}

static void hal_delay_ms(uint32_t ms)
{
    nrf_delay_ms(ms);
}

static bool sensor_init_zmod(void)
{
    int8_t ret;
    
    nrf_drv_twi_config_t config = NRF_DRV_TWI_DEFAULT_CONFIG;
    config.scl = TWI_SCL_PIN;
    config.sda = TWI_SDA_PIN;
    config.frequency = NRF_DRV_TWI_FREQ_100K;
    
    ret_code_t err = nrf_drv_twi_init(&m_twi, &config, NULL, NULL);
    if (err != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "TWI init failed: 0x%x\n", err);
        return false;
    }
    nrf_drv_twi_enable(&m_twi);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "TWI initialized\n");
    
    m_zmod_dev.i2c_addr = ZMOD4410_I2C_ADDR;
    m_zmod_dev.pid = ZMOD4410_PID;
    m_zmod_dev.init_conf = &zmod_iaq2_sensor_cfg[INIT];
    m_zmod_dev.meas_conf = &zmod_iaq2_sensor_cfg[MEASUREMENT];
    m_zmod_dev.prod_data = m_zmod_prod_data;
    m_zmod_dev.read = hal_i2c_read;
    m_zmod_dev.write = hal_i2c_write;
    m_zmod_dev.delay_ms = hal_delay_ms;
    
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing ZMOD4410...\n");
    ret = zmod4xxx_read_sensor_info(&m_zmod_dev);
    if (ret)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "ZMOD read sensor info failed: %d\n", ret);
        return false;
    }
    
    ret = zmod4xxx_prepare_sensor(&m_zmod_dev);
    if (ret)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "ZMOD prepare failed: %d\n", ret);
        return false;
    }
    
    ret = init_iaq_2nd_gen(&m_iaq_handle);
    if (ret)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "IAQ algorithm init failed: %d\n", ret);
        return false;
    }
    
    m_sample_count = 0;
    m_algorithm_stable = false;
    
    memset(&m_iaq_results, 0, sizeof(m_iaq_results));
    
    ret = zmod4xxx_start_measurement(&m_zmod_dev);
    if (ret)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "ZMOD start measurement failed: %d\n", ret);
        return false;
    }
    
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "ZMOD4410 initialized successfully\n");
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Thresholds - IAQ: %.2f, TVOC: %.2f, eCO2: %.0f\n",
          (double)IAQ_THRESHOLD, (double)TVOC_THRESHOLD, (double)ECO2_THRESHOLD);
    
    return true;
}

static void scheduled_meas_handler(void * p_event_data, uint16_t event_size)
{
    (void)p_event_data;
    (void)event_size;
    
    if (!m_sensor_initialized)
    {
        return;
    }
    
    int8_t ret;
    uint8_t status;
    
    ret = zmod4xxx_read_status(&m_zmod_dev, &status);
    if (ret)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Failed to read status: %d\n", ret);
        return;
    }
    
    if ((status & STATUS_SEQUENCER_RUNNING_MASK) != 0)
    {
        return;
    }
    
    ret = zmod4xxx_read_adc_result(&m_zmod_dev, m_zmod_adc_result);
    if (ret)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Failed to read ADC: %d\n", ret);
        goto start_next;
    }
    
    m_iaq_inputs.adc_result = m_zmod_adc_result;
    
    ret = calc_iaq_2nd_gen(&m_iaq_handle, &m_zmod_dev, NULL, &m_iaq_inputs, &m_iaq_results);
    
    m_sample_count++;
    
    if (ret == IAQ_2ND_GEN_STABILIZATION)
    {
        if (m_sample_count % 10 == 0)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Stabilizing... sample %u (algorithm warming up)\n", m_sample_count);
        }
        goto start_next;
    }
    else if (ret != IAQ_2ND_GEN_OK)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "IAQ calc error: %d\n", ret);
        goto start_next;
    }
    
    if (!m_algorithm_stable)
    {
        m_algorithm_stable = true;
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, 
              "*** Sensor stabilized after %u samples ***\n", m_sample_count);
    }
    
    if (!is_valid_float(m_iaq_results.iaq) || 
        !is_valid_float(m_iaq_results.tvoc) || 
        !is_valid_float(m_iaq_results.eco2))
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Invalid IAQ results (NaN or overflow)\n");
        goto start_next;
    }
    
    int16_t iaq_int = (int16_t)(m_iaq_results.iaq);
    int16_t iaq_frac = (int16_t)((m_iaq_results.iaq - (float)iaq_int) * 10.0f);
    if (iaq_frac < 0) iaq_frac = -iaq_frac;
    
    uint16_t tvoc_int = (uint16_t)(m_iaq_results.tvoc);
    uint16_t tvoc_frac = (uint16_t)((m_iaq_results.tvoc - (float)tvoc_int) * 100.0f);
    
    uint16_t eco2_int = (uint16_t)(m_iaq_results.eco2);
    
    if (iaq_int < 0 || iaq_int > 500 || eco2_int > 10000)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, 
              "IAQ values out of range: iaq=%d, eco2=%u\n", iaq_int, eco2_int);
        goto start_next;
    }
    
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, 
          "IAQ: %d.%d, TVOC: %u.%02u mg/m3, eCO2: %u ppm\n",
          iaq_int, iaq_frac,
          tvoc_int, tvoc_frac,
          eco2_int);
    
    if (should_publish_data(m_iaq_results.iaq, m_iaq_results.tvoc, m_iaq_results.eco2))
    {
        if (mesh_vendor_model_is_ready())
        {
            mesh_publish_sensor_values(
                m_iaq_results.iaq,
                m_iaq_results.tvoc,
                m_iaq_results.eco2
            );
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Published to mesh network\n");
        }
        else
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Vendor model not ready, skipping publish\n");
        }
    }
    
start_next:
    ret = zmod4xxx_start_measurement(&m_zmod_dev);
    if (ret)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Failed to start next measurement: %d\n", ret);
    }
}

static void meas_timer_handler(void * p_context)
{
    (void)p_context;
    app_sched_event_put(NULL, 0, scheduled_meas_handler);
}

void app_sensor_iaq_init(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "app_sensor_iaq_init\n");
    
    m_sensor_initialized = sensor_init_zmod();
    
    if (!m_sensor_initialized)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Sensor initialization failed\n");
        return;
    }
    
    ret_code_t rc = app_timer_create(&m_iaq_timer_id, 
                                     APP_TIMER_MODE_REPEATED, 
                                     meas_timer_handler);
    if (rc != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Timer create failed: 0x%x\n", rc);
        m_sensor_initialized = false;
        return;
    }
    
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "IAQ sensor initialized successfully\n");
}

void app_sensor_iaq_start(void)
{
    if (!m_sensor_initialized)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Cannot start: sensor not initialized\n");
        return;
    }
    
    if (m_timer_running)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Timer already running\n");
        return;
    }
    
    ret_code_t rc = app_timer_start(m_iaq_timer_id, 
                                    APP_TIMER_TICKS(APP_SENSOR_IAQ_MEAS_INTERVAL_MS), 
                                    NULL);
    if (rc != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Timer start failed: 0x%x\n", rc);
        return;
    }
    
    m_timer_running = true;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "IAQ measurements started (%u ms interval)\n", 
          APP_SENSOR_IAQ_MEAS_INTERVAL_MS);
}

void app_sensor_iaq_stop(void)
{
    app_timer_stop(m_iaq_timer_id);
    m_timer_running = false;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "IAQ measurements stopped\n");
}

void app_sensor_iaq_reset_thresholds(void)
{
    m_thresholds.first_reading = true;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Thresholds reset - next reading will publish\n");
}
