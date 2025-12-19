#ifndef LOGGING_COMPAT_H__
#define LOGGING_COMPAT_H__

/*
 * Lightweight compatibility wrapper to avoid linking the full nrf_log library.
 * Maps common NRF_LOG macros used in example code to the mesh logging macros
 * already present in the SDK (e.g. __LOG and __LOG_XB).
 *
 * Add this header to any app files that previously used NRF_LOG_*.
 *
 * NOTE: This intentionally maps only the small subset used by the example.
 * If you used other NRF_LOG macros, add mappings here similarly.
 */

#include "log.h" /* Mesh logging: defines __LOG, __LOG_XB, LOG_SRC_APP, LOG_LEVEL_xxx */

#ifndef LOG_SRC_APP
/* If your project uses different LOG_SRC names, define a fallback */
#define LOG_SRC_APP LOG_SRC_APPLICATION
#endif

/* Basic levels mapping (choose the mesh log level that matches desired verbosity) */
#define NRF_LOG_LEVEL_ERROR    LOG_LEVEL_ERROR
#define NRF_LOG_LEVEL_WARNING  LOG_LEVEL_WARNING
#define NRF_LOG_LEVEL_INFO     LOG_LEVEL_INFO
#define NRF_LOG_LEVEL_DEBUG    LOG_LEVEL_DBG3

/* Map NRF_LOG_INFO(fmt, ...) -> __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, fmt, ...) */
#define NRF_LOG_INFO(...)      __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, __VA_ARGS__)

/* Map NRF_LOG_WARNING(...) and NRF_LOG_ERROR(...) if present in code */
#define NRF_LOG_WARNING(...)   __LOG(LOG_SRC_APP, LOG_LEVEL_WARNING, __VA_ARGS__)
#define NRF_LOG_ERR(...)       __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, __VA_ARGS__)

/* Hexdump / raw buffers: NRF_LOG_HEXDUMP_INFO(ptr, len) -> __LOG_XB(...) */
#define NRF_LOG_HEXDUMP_INFO(p_buf, len)  __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, p_buf, len)
#define NRF_LOG_HEXDUMP_DEBUG(p_buf, len) __LOG_XB(LOG_SRC_APP, LOG_LEVEL_DBG3, p_buf, len)

/* If code used NRF_LOG_RAW or similar, map to simple info */
#define NRF_LOG_RAW_INFO(...)  __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, __VA_ARGS__)

/* If code references the nrf_log frontend symbols, define no-op placeholders.
 * These were causing the linker errors (nrf_log_frontend_std_0 etc.). We create
 * weak dummy symbols so the linker is satisfied (they won't get called by macros
 * used above, but other code that references them will link).
 *
 * We declare them here as weak functions so they won't conflict if real frontends
 * are added later.
 */

#ifdef __GNUC__
#define WEAK_ATTR __attribute__((weak))
#else
#define WEAK_ATTR
#endif

/* Provide empty weak symbols to satisfy any references to these names. */
void nrf_log_frontend_std_0(void) WEAK_ATTR;
void nrf_log_frontend_std_1(void) WEAK_ATTR;
void nrf_log_frontend_std_2(void) WEAK_ATTR;
void nrf_log_frontend_std_3(void) WEAK_ATTR;

/* If any code references the app logs data const symbol, provide a weak placeholder.
 * It is common to have a symbol like m_nrf_log_app_logs_data_const referenced by SDK logging.
 */
/* Dummy placeholder for SDK reference:
 * m_nrf_log_app_logs_data_const normally points to const section data.
 * We provide a 1-byte weak symbol to satisfy linking.
 */
extern const uint8_t m_nrf_log_app_logs_data_const WEAK_ATTR;
const uint8_t m_nrf_log_app_logs_data_const WEAK_ATTR = 0;


/* Minimal strerror helper used by some softdevice helpers. Provide a weak stub so
 * linking won't fail if nrf_strerror_get is referenced. If you want real strings,
 * add the real nrf_strerror.c to project instead.
 */
const char* nrf_strerror_get(uint32_t err_code) WEAK_ATTR;
const char* nrf_strerror_get(uint32_t err_code)
{
    (void)err_code;
    return "err";
}

/* app_scheduler compatibility: map app_sched_event_put and app_sched_init to mesh app_scheduler
 * if present. The mesh project already compiles app_scheduler.c; just declare the functions
 * here so that app_sensor_iaq.c can call them without including the other nrf header.
 *
 * If your project already has proper prototypes in app_scheduler.h, you can omit these.
 */
int app_sched_event_put(void * p_event_data, uint16_t event_size, void (*p_event_handler)(void * p_event_data, uint16_t event_size)) WEAK_ATTR;
int app_sched_event_put(void * p_event_data, uint16_t event_size, void (*p_event_handler)(void * p_event_data, uint16_t event_size))
{
    /* If real app_sched is linked, this weak stub will be overridden. */
    (void)p_event_data;
    (void)event_size;
    (void)p_event_handler;
    return 0;
}

void app_sched_init(void) WEAK_ATTR;
void app_sched_init(void) { /* no-op stub; real implementation will override if present */ }

#endif /* LOGGING_COMPAT_H__ */
