// SPDX-License-Identifier: MIT
//
// A camera sensor described by a Python dict: see sensor_table.c.

#pragma once

#include "py/obj.h"
#include "esp_cam_sensor.h"

// Copy a description into the C heap; *addr gets its SCCB address. Raises on
// a malformed description.
void *cameraif_table_parse(mp_obj_t desc, uint8_t *addr);

// Free a parsed table that never became a device.
void cameraif_table_free(void *table);

// The device, if the chip answering on config->sccb_handle has the table's
// ID; NULL otherwise. Owns the table either way.
esp_cam_sensor_device_t *cameraif_table_detect(void *table, esp_cam_sensor_config_t *config);
