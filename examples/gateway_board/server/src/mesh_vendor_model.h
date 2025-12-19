#ifndef MESH_VENDOR_MODEL_H__
#define MESH_VENDOR_MODEL_H__

#include <stdint.h>
#include <stdbool.h>
#include "access.h"

uint32_t mesh_vendor_model_init(void);
void mesh_publish_sensor_values(float iaq, float tvoc, float eco2);
access_model_handle_t mesh_vendor_model_handle_get(void);
bool mesh_vendor_model_is_ready(void);

/* Call this when config server reports publication was set */
void mesh_vendor_model_publication_set(void);

#endif /* MESH_VENDOR_MODEL_H__ */
