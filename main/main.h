/*
 * ESP32-C6 LD2450 Zigbee Presence Sensor
 * Copyright (C) 2025 squwal
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "esp_zigbee_core.h"
#include "zcl_utility.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false                                /* enable the install code policy for security */
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN        /* aging timeout of device */
#define ED_KEEP_ALIVE                   3000                                 /* 3000 millisecond */
#define HA_ESP_LIGHT_ENDPOINT           10                                   /* esp light bulb device endpoint, used to process light controlling commands */

#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use in the example */

/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME "\x09""ESPRESSIF"      /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER "\x07"CONFIG_IDF_TARGET /* Customized model identifier */

/* Definitions for the LD2450 custom cluster */
#define ESP_ZB_LD2450_CUSTOM_CLUSTER_ID                  0xFF00  // Custom cluster ID (range 0xFC00-0xFFFF reserved for custom clusters)

// Custom cluster attributes
#define ESP_ZB_LD2450_ATTR_TARGET1_PRESENCE_ID           0x0000  // Target 1 presence (boolean)
#define ESP_ZB_LD2450_ATTR_TARGET1_DISTANCE_ID           0x0001  // Target 1 distance in mm (uint16)
#define ESP_ZB_LD2450_ATTR_TARGET1_X_ID                  0x0002  // Target 1 X coordinate in mm (int16)
#define ESP_ZB_LD2450_ATTR_TARGET1_Y_ID                  0x0003  // Target 1 Y coordinate in mm (int16)

#define ESP_ZB_LD2450_ATTR_TARGET2_PRESENCE_ID           0x0010  // Target 2 presence (boolean)
#define ESP_ZB_LD2450_ATTR_TARGET2_DISTANCE_ID           0x0011  // Target 2 distance in mm (uint16)
#define ESP_ZB_LD2450_ATTR_TARGET2_X_ID                  0x0012  // Target 2 X coordinate in mm (int16)
#define ESP_ZB_LD2450_ATTR_TARGET2_Y_ID                  0x0013  // Target 2 Y coordinate in mm (int16)

#define ESP_ZB_LD2450_ATTR_TARGET3_PRESENCE_ID           0x0020  // Target 3 presence (boolean)
#define ESP_ZB_LD2450_ATTR_TARGET3_DISTANCE_ID           0x0021  // Target 3 distance in mm (uint16)
#define ESP_ZB_LD2450_ATTR_TARGET3_X_ID                  0x0022  // Target 3 X coordinate in mm (int16)
#define ESP_ZB_LD2450_ATTR_TARGET3_Y_ID                  0x0023  // Target 3 Y coordinate in mm (int16)

#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ED_AGING_TIMEOUT,                         \
            .keep_alive = ED_KEEP_ALIVE,                            \
        },                                                          \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }

void sensor_reading_task(void *pvParameters);
void reportAttribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, void *value, uint8_t value_length);