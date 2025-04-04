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

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include <math.h>  // For log10f
#include "string.h"
#include "esp_log.h"
#include "ld2450.h"
#include "bh1750.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "light_driver.h"
#include "main.h"

static const char *TAG = "presence_sensor";

volatile static bool zigbee_connected = false;

#define DEFINE_PSTRING(var, str)   \
    const struct                   \
    {                              \
        unsigned char len;         \
        char content[sizeof(str)]; \
    }(var) = {sizeof(str) - 1, (str)}

// GPIO configuration for LD2450
#define ESP32_UART_RX_PIN GPIO_NUM_4  // GPIO4: ESP32 RX connected to LD2450 TX
#define ESP32_UART_TX_PIN GPIO_NUM_5  // GPIO5: ESP32 TX connected to LD2450 RX

// GPIO configuration for BH1750
#define BH1750_SDA_PIN GPIO_NUM_6
#define BH1750_SCL_PIN GPIO_NUM_7
#define BH1750_I2C_PORT I2C_NUM_0

// Illuminance cluster configuration
#define SENSOR_ILLUMINANCE_MEASUREMENT_VALUE_DEFAULT     0x0000
#define REPORTING_INTERVAL                             5000    // 5 seconds

// Structure for sharing data between tasks
typedef struct {
    SemaphoreHandle_t mutex;
    uint16_t illuminance;
    bool presence_detected;
} sensor_data_t;

static sensor_data_t shared_sensor_data = {
    .mutex = NULL,
    .illuminance = 0,
    .presence_detected = false
};

// Function declarations
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask);
static esp_err_t deferred_driver_init(void);
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message);
static void sensor_task(void *pvParameters);
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message);
static void binding_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx);
static void bind_custom_clusters_to_coordinator(void);
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct);
void reportAttribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, void *value, uint8_t value_length);
void esp_zb_task(void *pvParameters);

// Sensor reading task
void sensor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor task started");
    
    bh1750_data_t light_data = {0};
    ld2450_data_t radar_data = {0};
    esp_err_t ret;
    
    // I2C initialization
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BH1750_SDA_PIN,
        .scl_io_num = BH1750_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    ESP_ERROR_CHECK(i2c_param_config(BH1750_I2C_PORT, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(BH1750_I2C_PORT, i2c_config.mode, 0, 0, 0));

    // BH1750 initialization
    ESP_LOGI(TAG, "Initializing BH1750 sensor...");
    ret = bh1750_detect(BH1750_I2C_PORT, BH1750_I2C_ADDRESS_LO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BH1750 sensor not found!");
        vTaskDelete(NULL);
        return;
    }

    bh1750_config_t bh1750_config = {
        .i2c_port = BH1750_I2C_PORT,
        .i2c_address = BH1750_I2C_ADDRESS_LO,
        .sda_pin = BH1750_SDA_PIN,
        .scl_pin = BH1750_SCL_PIN,
        .timeout_ms = 1000
    };
    ESP_ERROR_CHECK(bh1750_init(&bh1750_config));
    ESP_ERROR_CHECK(bh1750_set_mode(BH1750_CMD_CONT_HRES));

    // LD2450 initialization
    ESP_LOGI(TAG, "Initializing LD2450 sensor...");
    ld2450_config_t ld2450_config = {
        .uart_num = UART_NUM_1,
        .tx_pin = ESP32_UART_TX_PIN,
        .rx_pin = ESP32_UART_RX_PIN
    };
    ret = ld2450_init(&ld2450_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LD2450 sensor initialization failed!");
        vTaskDelete(NULL);
        return;
    }
    
    // Wait for Zigbee network initialization
    while (!zigbee_connected) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    ESP_LOGI(TAG, "Zigbee network initialized, starting sensor readings");
    
    while (1) {
        // Reading BH1750
        ret = bh1750_read_data(&light_data);
        if (ret == ESP_OK && light_data.sensor_ok) {
            // Converting lux value to Zigbee format for illuminance
            // Formula: value = 10000 * log10(illuminance in lux) + 1
            uint16_t zigbee_illuminance;
            
            if (light_data.light_level > 0) {
                zigbee_illuminance = (uint16_t)(10000.0f * log10f((float)light_data.light_level) + 1.0f);
            } else {
                // If brightness is 0 or negative, use the minimum value
                zigbee_illuminance = 1;
            }
            
            // Update brightness attribute with the new reportAttribute function
            reportAttribute(HA_ESP_LIGHT_ENDPOINT, 
                          ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT, 
                          ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID, 
                          &zigbee_illuminance, 
                          sizeof(zigbee_illuminance));
            
            //ESP_LOGI(TAG, "Luminosity measured: %d lux (Zigbee format: %d)", light_data.light_level, zigbee_illuminance);
        } else {
            ESP_LOGE(TAG, "Error reading BH1750 sensor");
        }
        
        // Reading LD2450
        ret = ld2450_read_data(&radar_data);
        if (ret == ESP_OK) {         
            // Update custom cluster attributes for each target
            for (int i = 0; i < 3; i++) {
                bool target_presence = false;
                uint16_t target_distance = 0;
                
                // Check if target is active
                if (i < radar_data.num_targets) {
                    // Active target
                    target_presence = true;
                    
                    // Calculate distance in mm
                    float x_meters = radar_data.targets[i].x / 1000.0f;
                    float y_meters = radar_data.targets[i].y / 1000.0f;
                    float distance_meters = sqrtf(x_meters * x_meters + y_meters * y_meters);
                    target_distance = (uint16_t)(distance_meters * 1000.0f); // Conversion en mm
                    
                    ESP_LOGD(TAG, "Target %d: Distance = %.2f m, X = %d mm, Y = %d mm", 
                             i+1, distance_meters, radar_data.targets[i].x, radar_data.targets[i].y);
                }
                
                // Update attributes for this target
                if (i == 0) {
                    // Target 1
                    reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_LD2450_CUSTOM_CLUSTER_ID, 
                                  ESP_ZB_LD2450_ATTR_TARGET1_PRESENCE_ID, &target_presence, sizeof(target_presence));
                    
                    reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_LD2450_CUSTOM_CLUSTER_ID, 
                                  ESP_ZB_LD2450_ATTR_TARGET1_DISTANCE_ID, &target_distance, sizeof(target_distance));
                    
                    reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_LD2450_CUSTOM_CLUSTER_ID, 
                                  ESP_ZB_LD2450_ATTR_TARGET1_X_ID, &radar_data.targets[i].x, sizeof(radar_data.targets[i].x));
                    
                    reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_LD2450_CUSTOM_CLUSTER_ID, 
                                  ESP_ZB_LD2450_ATTR_TARGET1_Y_ID, &radar_data.targets[i].y, sizeof(radar_data.targets[i].y));
                } 
                else if (i == 1) {
                    // Cible 2
                    reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_LD2450_CUSTOM_CLUSTER_ID, 
                                  ESP_ZB_LD2450_ATTR_TARGET2_PRESENCE_ID, &target_presence, sizeof(target_presence));
                    
                    reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_LD2450_CUSTOM_CLUSTER_ID, 
                                  ESP_ZB_LD2450_ATTR_TARGET2_DISTANCE_ID, &target_distance, sizeof(target_distance));
                    
                    reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_LD2450_CUSTOM_CLUSTER_ID, 
                                  ESP_ZB_LD2450_ATTR_TARGET2_X_ID, &radar_data.targets[i].x, sizeof(radar_data.targets[i].x));
                    
                    reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_LD2450_CUSTOM_CLUSTER_ID, 
                                  ESP_ZB_LD2450_ATTR_TARGET2_Y_ID, &radar_data.targets[i].y, sizeof(radar_data.targets[i].y));
                }
                else if (i == 2) {
                    // Cible 3
                    reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_LD2450_CUSTOM_CLUSTER_ID, 
                                  ESP_ZB_LD2450_ATTR_TARGET3_PRESENCE_ID, &target_presence, sizeof(target_presence));
                    
                    reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_LD2450_CUSTOM_CLUSTER_ID, 
                                  ESP_ZB_LD2450_ATTR_TARGET3_DISTANCE_ID, &target_distance, sizeof(target_distance));
                    
                    reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_LD2450_CUSTOM_CLUSTER_ID, 
                                  ESP_ZB_LD2450_ATTR_TARGET3_X_ID, &radar_data.targets[i].x, sizeof(radar_data.targets[i].x));
                    
                    reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_LD2450_CUSTOM_CLUSTER_ID, 
                                  ESP_ZB_LD2450_ATTR_TARGET3_Y_ID, &radar_data.targets[i].y, sizeof(radar_data.targets[i].y));
                }
            }
        } else if (ret == ESP_ERR_TIMEOUT) {
            // Timeout est normal si aucune donnée n'est disponible, ne pas afficher d'erreur
            ESP_LOGD(TAG, "No data available from LD2450 sensor (timeout)");
            
            // Mettre à jour les attributs pour indiquer qu'aucune cible n'est présente
            bool target_presence = false;
            uint16_t target_distance = 0;
            int16_t target_coord = 0;
            
            // Cible 1
            reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_LD2450_CUSTOM_CLUSTER_ID, 
                          ESP_ZB_LD2450_ATTR_TARGET1_PRESENCE_ID, &target_presence, sizeof(target_presence));
            
            // Cible 2
            reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_LD2450_CUSTOM_CLUSTER_ID, 
                          ESP_ZB_LD2450_ATTR_TARGET2_PRESENCE_ID, &target_presence, sizeof(target_presence));
            
            // Cible 3
            reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_LD2450_CUSTOM_CLUSTER_ID, 
                          ESP_ZB_LD2450_ATTR_TARGET3_PRESENCE_ID, &target_presence, sizeof(target_presence));
        } else {
            // Autres erreurs
            ESP_LOGW(TAG, "Error reading LD2450 sensor: %d", ret);
            
            // Mettre à jour les attributs pour indiquer qu'aucune cible n'est présente
            bool target_presence = false;
            uint16_t target_distance = 0;
            int16_t target_coord = 0;
            
            // Cible 1
            reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_LD2450_CUSTOM_CLUSTER_ID, 
                          ESP_ZB_LD2450_ATTR_TARGET1_PRESENCE_ID, &target_presence, sizeof(target_presence));
            
            // Cible 2
            reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_LD2450_CUSTOM_CLUSTER_ID, 
                          ESP_ZB_LD2450_ATTR_TARGET2_PRESENCE_ID, &target_presence, sizeof(target_presence));
            
            // Cible 3
            reportAttribute(HA_ESP_LIGHT_ENDPOINT, ESP_ZB_LD2450_CUSTOM_CLUSTER_ID, 
                          ESP_ZB_LD2450_ATTR_TARGET3_PRESENCE_ID, &target_presence, sizeof(target_presence));
        }
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Lecture toutes les 250ms
    }
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK)
        {
            ESP_LOGI(TAG, "Start network steering");
            ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        }
        else
        {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            zigbee_connected = true;
            
            // Après la connexion réussie au réseau, effectuer le binding
            bind_custom_clusters_to_coordinator();
            
            xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    case ESP_ZB_NWK_SIGNAL_DEVICE_ASSOCIATED:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Network association successful");
            zigbee_connected = true;
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGI(TAG, "Left network, starting network steering");
        zigbee_connected = false;
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

// Fonction améliorée pour mettre à jour les attributs avec verrous Zigbee
void reportAttribute(uint8_t endpoint, uint16_t clusterID, uint16_t attributeID, void *value, uint8_t value_length)
{
    /*ESP_LOGI(TAG, "Setting attribute - endpoint: %d, cluster: 0x%04x, attribute: 0x%04x", 
             endpoint, clusterID, attributeID);*/
    
    if (value == NULL) {
        ESP_LOGE(TAG, "Value pointer is NULL");
        return;
    }
    
    // Utilisation des verrous Zigbee pour protéger l'accès à la pile
    esp_zb_lock_acquire(portMAX_DELAY);
    
    // Mise à jour directe de l'attribut avec esp_zb_zcl_set_attribute_val
    esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(
        endpoint, 
        clusterID, 
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, 
        attributeID, 
        value, 
        false);
    
    esp_zb_lock_release();
    
    if (status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        ESP_LOGW(TAG, "Failed to set attribute value, status: 0x%02x", status);
    } else {
        //ESP_LOGI(TAG, "Attribute value updated successfully");
 /*       
        // Affichage de la valeur (pour debug)
        if (value_length == 2) {
            ESP_LOGI(TAG, "New value: 0x%04x (%d)", *(uint16_t*)value, *(uint16_t*)value);
        } else if (value_length == 1) {
            ESP_LOGI(TAG, "New value: 0x%02x (%d)", *(uint8_t*)value, *(uint8_t*)value);
        }
*/
    }
}

// Fonction callback pour le binding
static void binding_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
{
    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Binding to coordinator successful for LD2450 custom cluster");
        
        // Après un binding réussi, on peut configurer le delta pour les attributs booléens
        // si ce n'est pas déjà fait dans la configuration initiale
        esp_zb_zcl_reporting_info_t *reporting_info = (esp_zb_zcl_reporting_info_t *)user_ctx;
        if (reporting_info) {
            ESP_LOGI(TAG, "Updating reporting configuration for attribute 0x%04x", reporting_info->attr_id);
            esp_zb_zcl_update_reporting_info(reporting_info);
        }
    } else {
        ESP_LOGW(TAG, "Binding to coordinator failed with status: %d", zdo_status);
    }
}

// Fonction pour effectuer le binding au coordinateur
static void bind_custom_clusters_to_coordinator(void)
{
    ESP_LOGI(TAG, "Binding custom LD2450 cluster to coordinator");
    
    // Création d'une structure de reporting pour être utilisée après le binding
    static esp_zb_zcl_reporting_info_t presence_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET1_PRESENCE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,          // Intervalle minimum en secondes
        .u.send_info.max_interval = 43200,         // Intervalle maximum en secondes
        .u.send_info.def_min_interval = 1,      // Intervalle minimum par défaut
        .u.send_info.def_max_interval = 43200,     // Intervalle maximum par défaut
        .u.send_info.delta.u8 = 1,              // Pour les booléens, un changement de 1 signifie tout changement
    };
    
    esp_zb_zdo_bind_req_param_t bind_req;
    
    // Paramètres source (notre appareil)
    esp_zb_get_long_address(bind_req.src_address);
    bind_req.src_endp = HA_ESP_LIGHT_ENDPOINT;
    bind_req.cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID;
    
    // Mode d'adressage destination (coordinateur)
    bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
    
    // Obtention de l'adresse IEEE du coordinateur (adresse 0)
    esp_zb_ieee_address_by_short(0, bind_req.dst_address_u.addr_long);
    bind_req.dst_endp = HA_ESP_LIGHT_ENDPOINT; // Utiliser le même endpoint
    bind_req.req_dst_addr = esp_zb_get_short_address(); // Notre adresse (pour l'envoi de la requête)
    
    // Envoi de la requête de binding avec la structure de reporting comme contexte
    esp_zb_zdo_device_bind_req(&bind_req, binding_cb, &presence_reporting_info);
    
    // Créer et envoyer des bindings pour les autres attributs importants
    // Par exemple, pour target1_distance
    bind_req.cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID; // Même cluster
    
    static esp_zb_zcl_reporting_info_t distance_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET1_DISTANCE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 3,
        .u.send_info.max_interval = 43200,
        .u.send_info.def_min_interval = 3,
        .u.send_info.def_max_interval = 43200,
        .u.send_info.delta.u16 = 400,  // 400mm = 40cm
    };
    
    // Nous réutilisons la même structure bind_req avec juste un contexte différent
    esp_zb_zdo_device_bind_req(&bind_req, binding_cb, &distance_reporting_info);
    
    // Configuration du reporting pour la coordonnée X
    static esp_zb_zcl_reporting_info_t x_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET1_X_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 3,
        .u.send_info.max_interval = 43200,
        .u.send_info.def_min_interval = 3,
        .u.send_info.def_max_interval = 43200,
        .u.send_info.delta.s16 = 400,  // 400mm = 40cm
    };
    
    // Nous réutilisons la même structure bind_req avec juste un contexte différent
    esp_zb_zdo_device_bind_req(&bind_req, binding_cb, &x_reporting_info);
    
    // Configuration du reporting pour la coordonnée Y
    static esp_zb_zcl_reporting_info_t y_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET1_Y_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 3,
        .u.send_info.max_interval = 43200,
        .u.send_info.def_min_interval = 3,
        .u.send_info.def_max_interval = 43200,
        .u.send_info.delta.s16 = 400,  // 400mm = 40cm
    };
    
    // Nous réutilisons la même structure bind_req avec juste un contexte différent
    esp_zb_zdo_device_bind_req(&bind_req, binding_cb, &y_reporting_info);
    
    // ---------------------- CIBLE 2 ----------------------
    // Configuration du reporting pour la présence de la cible 2
    static esp_zb_zcl_reporting_info_t presence2_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET2_PRESENCE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,          // Intervalle minimum en secondes
        .u.send_info.max_interval = 43200,         // Intervalle maximum en secondes
        .u.send_info.def_min_interval = 1,      // Intervalle minimum par défaut
        .u.send_info.def_max_interval = 43200,     // Intervalle maximum par défaut
        .u.send_info.delta.u8 = 1,
    };
    
    esp_zb_zdo_device_bind_req(&bind_req, binding_cb, &presence2_reporting_info);
    
    // Configuration du reporting pour la distance de la cible 2
    static esp_zb_zcl_reporting_info_t distance2_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET2_DISTANCE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 3,
        .u.send_info.max_interval = 43200,
        .u.send_info.def_min_interval = 3,
        .u.send_info.def_max_interval = 43200,
        .u.send_info.delta.u16 = 400,  // 400mm = 40cm
    };
    
    esp_zb_zdo_device_bind_req(&bind_req, binding_cb, &distance2_reporting_info);
    
    // Configuration du reporting pour la coordonnée X de la cible 2
    static esp_zb_zcl_reporting_info_t x2_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET2_X_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 3,
        .u.send_info.max_interval = 43200,
        .u.send_info.def_min_interval = 3,
        .u.send_info.def_max_interval = 43200,
        .u.send_info.delta.s16 = 400,  // 400mm = 40cm
    };
    
    esp_zb_zdo_device_bind_req(&bind_req, binding_cb, &x2_reporting_info);
    
    // Configuration du reporting pour la coordonnée Y de la cible 2
    static esp_zb_zcl_reporting_info_t y2_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET2_Y_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 3,
        .u.send_info.max_interval = 43200,
        .u.send_info.def_min_interval = 3,
        .u.send_info.def_max_interval = 43200,
        .u.send_info.delta.s16 = 400,  // 400mm = 40cm
    };
    
    esp_zb_zdo_device_bind_req(&bind_req, binding_cb, &y2_reporting_info);
    
    // ---------------------- CIBLE 3 ----------------------
    // Configuration du reporting pour la présence de la cible 3
    static esp_zb_zcl_reporting_info_t presence3_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET3_PRESENCE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,          // Intervalle minimum en secondes
        .u.send_info.max_interval = 43200,         // Intervalle maximum en secondes
        .u.send_info.def_min_interval = 1,      // Intervalle minimum par défaut
        .u.send_info.def_max_interval = 43200,     // Intervalle maximum par défaut
        .u.send_info.delta.u8 = 1,
    };
    
    esp_zb_zdo_device_bind_req(&bind_req, binding_cb, &presence3_reporting_info);
    
    // Configuration du reporting pour la distance de la cible 3
    static esp_zb_zcl_reporting_info_t distance3_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET3_DISTANCE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 3,
        .u.send_info.max_interval = 43200,
        .u.send_info.def_min_interval = 3,
        .u.send_info.def_max_interval = 43200,
        .u.send_info.delta.u16 = 400,  // 400mm = 40cm
    };
    
    esp_zb_zdo_device_bind_req(&bind_req, binding_cb, &distance3_reporting_info);
    
    // Configuration du reporting pour la coordonnée X de la cible 3
    static esp_zb_zcl_reporting_info_t x3_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET3_X_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 3,
        .u.send_info.max_interval = 43200,
        .u.send_info.def_min_interval = 3,
        .u.send_info.def_max_interval = 43200,
        .u.send_info.delta.s16 = 400,  // 400mm = 40cm
    };
    
    esp_zb_zdo_device_bind_req(&bind_req, binding_cb, &x3_reporting_info);
    
    // Configuration du reporting pour la coordonnée Y de la cible 3
    static esp_zb_zcl_reporting_info_t y3_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET3_Y_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 3,
        .u.send_info.max_interval = 43200,
        .u.send_info.def_min_interval = 3,
        .u.send_info.def_max_interval = 43200,
        .u.send_info.delta.s16 = 400,  // 400mm = 40cm
    };
    
    esp_zb_zdo_device_bind_req(&bind_req, binding_cb, &y3_reporting_info);
}

// Tâche Zigbee modifiée
void esp_zb_task(void *pvParameters)
{
    /* Initialize Zigbee stack with default config */
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // ------------------------------ Cluster BASIC ------------------------------
    esp_zb_basic_cluster_cfg_t basic_cluster_cfg = {
        .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = 0x03,
    };
    uint32_t ApplicationVersion = 0x0001;
    uint32_t StackVersion = 0x0002;
    uint32_t HWVersion = 0x0002;
    DEFINE_PSTRING(ManufacturerName, "SquwalInc");
    DEFINE_PSTRING(ModelIdentifier, "ESP-C6 Presency");
    DEFINE_PSTRING(DateCode, "20250223");

    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_basic_cluster_create(&basic_cluster_cfg);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &ApplicationVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &StackVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &HWVersion);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)&ManufacturerName);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)&ModelIdentifier);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, (void *)&DateCode);

    // ------------------------------ Cluster IDENTIFY ------------------------------
    esp_zb_identify_cluster_cfg_t identify_cluster_cfg = {
        .identify_time = 0,
    };
    esp_zb_attribute_list_t *esp_zb_identify_cluster = esp_zb_identify_cluster_create(&identify_cluster_cfg);

    // ------------------------------ Cluster LIGHT ------------------------------
    esp_zb_on_off_cluster_cfg_t on_off_cfg = {
        .on_off = 0,
    };
    esp_zb_attribute_list_t *esp_zb_on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);

    // ------------------------------ Cluster Illuminance ------------------------------
    // Les valeurs min et max sont au format Zigbee
    // 1 lux = 10000*log10(1)+1 = 1 (min)
    // 100000 lux (plein soleil) = 10000*log10(100000)+1 = 50001 (max)
    esp_zb_illuminance_meas_cluster_cfg_t illuminance_cluster_cfg = {
        .measured_value = 1,      // Valeur initiale (1 lux)
        .min_value = 1,           // Valeur minimale (1 lux)
        .max_value = 50001,       // Valeur maximale (100000 lux)
    };
    esp_zb_attribute_list_t *esp_zb_illuminance_cluster = esp_zb_illuminance_meas_cluster_create(&illuminance_cluster_cfg);

    // ------------------------------ Cluster personnalisé LD2450 ------------------------------
    // Création des attributs pour la cible 1
    esp_zb_attribute_list_t *esp_zb_target_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_LD2450_CUSTOM_CLUSTER_ID);
    
    // Attributs pour la cible 1
    bool target1_presence = false;
    uint16_t target1_distance = 0;
    int16_t target1_x = 0;
    int16_t target1_y = 0;
    
    esp_zb_custom_cluster_add_custom_attr(esp_zb_target_cluster, ESP_ZB_LD2450_ATTR_TARGET1_PRESENCE_ID, 
                        ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &target1_presence);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_target_cluster, ESP_ZB_LD2450_ATTR_TARGET1_DISTANCE_ID, 
                        ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &target1_distance);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_target_cluster, ESP_ZB_LD2450_ATTR_TARGET1_X_ID, 
                        ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &target1_x);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_target_cluster, ESP_ZB_LD2450_ATTR_TARGET1_Y_ID, 
                        ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &target1_y);
    
    // Attributs pour la cible 2
    bool target2_presence = false;
    uint16_t target2_distance = 0;
    int16_t target2_x = 0;
    int16_t target2_y = 0;
    
    esp_zb_custom_cluster_add_custom_attr(esp_zb_target_cluster, ESP_ZB_LD2450_ATTR_TARGET2_PRESENCE_ID, 
                        ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &target2_presence);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_target_cluster, ESP_ZB_LD2450_ATTR_TARGET2_DISTANCE_ID, 
                        ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &target2_distance);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_target_cluster, ESP_ZB_LD2450_ATTR_TARGET2_X_ID, 
                        ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &target2_x);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_target_cluster, ESP_ZB_LD2450_ATTR_TARGET2_Y_ID, 
                        ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &target2_y);
    
    // Attributs pour la cible 3
    bool target3_presence = false;
    uint16_t target3_distance = 0;
    int16_t target3_x = 0;
    int16_t target3_y = 0;
    
    esp_zb_custom_cluster_add_custom_attr(esp_zb_target_cluster, ESP_ZB_LD2450_ATTR_TARGET3_PRESENCE_ID, 
                        ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &target3_presence);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_target_cluster, ESP_ZB_LD2450_ATTR_TARGET3_DISTANCE_ID, 
                        ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &target3_distance);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_target_cluster, ESP_ZB_LD2450_ATTR_TARGET3_X_ID, 
                        ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &target3_x);
    esp_zb_custom_cluster_add_custom_attr(esp_zb_target_cluster, ESP_ZB_LD2450_ATTR_TARGET3_Y_ID, 
                        ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &target3_y);
    
    // ------------------------------ Create cluster list ------------------------------
    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_identify_cluster(esp_zb_cluster_list, esp_zb_identify_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_on_off_cluster(esp_zb_cluster_list, esp_zb_on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_illuminance_meas_cluster(esp_zb_cluster_list, esp_zb_illuminance_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    esp_zb_cluster_list_add_custom_cluster(esp_zb_cluster_list, esp_zb_target_cluster,  ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    
    // ------------------------------ Create endpoint list ------------------------------
    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = HA_ESP_LIGHT_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
    };

    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);

    // ------------------------------ Register Device ------------------------------
    esp_zb_device_register(esp_zb_ep_list);
    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);

    // ------------------------------ Configure Attribute Reporting ------------------------------
    // Configuration du reporting automatique pour l'attribut de luminosité
    esp_zb_zcl_reporting_info_t illuminance_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,          // Intervalle minimum en secondes
        .u.send_info.max_interval = 43200,         // Intervalle maximum en secondes
        .u.send_info.def_min_interval = 1,      // Intervalle minimum par défaut
        .u.send_info.def_max_interval = 43200,     // Intervalle maximum par défaut
        .u.send_info.delta.u16 = 400,            // Seuil de changement pour déclencher un rapport
    };
    
    ESP_LOGI(TAG, "Configuring automatic reporting for illuminance attribute");
    esp_zb_zcl_update_reporting_info(&illuminance_reporting_info);

    
    // Configuration du reporting automatique pour les attributs du cluster LD2450
    
    // Target 1 presence
    esp_zb_zcl_reporting_info_t target1_presence_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET1_PRESENCE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,          // Intervalle minimum en secondes
        .u.send_info.max_interval = 43200,          // Intervalle maximum en secondes
        .u.send_info.def_min_interval = 1,      // Intervalle minimum par défaut
        .u.send_info.def_max_interval = 43200,      // Intervalle maximum par défaut
        .u.send_info.delta.u8 = 1, // Pour les booléens, un changement de 1 signifie tout changement
    };
    
    // Target 1 distance
    esp_zb_zcl_reporting_info_t target1_distance_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET1_DISTANCE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,          // Intervalle minimum en secondes
        .u.send_info.max_interval = 43200,          // Intervalle maximum en secondes
        .u.send_info.def_min_interval = 1,      // Intervalle minimum par défaut
        .u.send_info.def_max_interval = 43200,      // Intervalle maximum par défaut
        .u.send_info.delta.u16 = 400,            // Seuil de changement pour déclencher un rapport (400mm = 40cm)
    };
    
    // Target 1 X coordinate
    esp_zb_zcl_reporting_info_t target1_x_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET1_X_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,          // Intervalle minimum en secondes
        .u.send_info.max_interval = 43200,          // Intervalle maximum en secondes
        .u.send_info.def_min_interval = 1,      // Intervalle minimum par défaut
        .u.send_info.def_max_interval = 43200,      // Intervalle maximum par défaut
        .u.send_info.delta.s16 = 400,            // Seuil de changement pour déclencher un rapport (400mm = 40cm)
    };
    
    // Target 1 Y coordinate
    esp_zb_zcl_reporting_info_t target1_y_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET1_Y_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,          // Intervalle minimum en secondes
        .u.send_info.max_interval = 43200,          // Intervalle maximum en secondes
        .u.send_info.def_min_interval = 1,      // Intervalle minimum par défaut
        .u.send_info.def_max_interval = 43200,      // Intervalle maximum par défaut
        .u.send_info.delta.s16 = 400,            // Seuil de changement pour déclencher un rapport (400mm = 40cm)
    };
    
    ESP_LOGI(TAG, "Configuring automatic reporting for LD2450 target 1 attributes");
    esp_zb_zcl_update_reporting_info(&target1_presence_reporting_info);
    esp_zb_zcl_update_reporting_info(&target1_distance_reporting_info);
    esp_zb_zcl_update_reporting_info(&target1_x_reporting_info);
    esp_zb_zcl_update_reporting_info(&target1_y_reporting_info);
    
    // Target 2 presence
    esp_zb_zcl_reporting_info_t target2_presence_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET2_PRESENCE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,          // Intervalle minimum en secondes
        .u.send_info.max_interval = 43200,          // Intervalle maximum en secondes
        .u.send_info.def_min_interval = 1,      // Intervalle minimum par défaut
        .u.send_info.def_max_interval = 43200,      // Intervalle maximum par défaut
        .u.send_info.delta.u8 = 1, // Pour les booléens, un changement de 1 signifie tout changement
    };
    
    // Target 2 distance
    esp_zb_zcl_reporting_info_t target2_distance_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET2_DISTANCE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,          // Intervalle minimum en secondes
        .u.send_info.max_interval = 43200,          // Intervalle maximum en secondes
        .u.send_info.def_min_interval = 1,      // Intervalle minimum par défaut
        .u.send_info.def_max_interval = 43200,      // Intervalle maximum par défaut
        .u.send_info.delta.u16 = 400,            // Seuil de changement pour déclencher un rapport (400mm = 40cm)
    };
    
    // Target 2 X coordinate
    esp_zb_zcl_reporting_info_t target2_x_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET2_X_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,          // Intervalle minimum en secondes
        .u.send_info.max_interval = 43200,          // Intervalle maximum en secondes
        .u.send_info.def_min_interval = 1,      // Intervalle minimum par défaut
        .u.send_info.def_max_interval = 43200,      // Intervalle maximum par défaut
        .u.send_info.delta.s16 = 400,            // Seuil de changement pour déclencher un rapport (400mm = 40cm)
    };
    
    // Target 2 Y coordinate
    esp_zb_zcl_reporting_info_t target2_y_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET2_Y_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,          // Intervalle minimum en secondes
        .u.send_info.max_interval = 43200,          // Intervalle maximum en secondes
        .u.send_info.def_min_interval = 1,      // Intervalle minimum par défaut
        .u.send_info.def_max_interval = 43200,      // Intervalle maximum par défaut
        .u.send_info.delta.s16 = 400,            // Seuil de changement pour déclencher un rapport (400mm = 40cm)
    };
    
    ESP_LOGI(TAG, "Configuring automatic reporting for LD2450 target 2 attributes");
    esp_zb_zcl_update_reporting_info(&target2_presence_reporting_info);
    esp_zb_zcl_update_reporting_info(&target2_distance_reporting_info);
    esp_zb_zcl_update_reporting_info(&target2_x_reporting_info);
    esp_zb_zcl_update_reporting_info(&target2_y_reporting_info);
    
    // Target 3 presence
    esp_zb_zcl_reporting_info_t target3_presence_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET3_PRESENCE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,          // Intervalle minimum en secondes
        .u.send_info.max_interval = 43200,          // Intervalle maximum en secondes
        .u.send_info.def_min_interval = 1,      // Intervalle minimum par défaut
        .u.send_info.def_max_interval = 43200,      // Intervalle maximum par défaut
        .u.send_info.delta.u8 = 1, // Pour les booléens, un changement de 1 signifie tout changement
    };
    
    // Target 3 distance
    esp_zb_zcl_reporting_info_t target3_distance_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET3_DISTANCE_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,          // Intervalle minimum en secondes
        .u.send_info.max_interval = 43200,          // Intervalle maximum en secondes
        .u.send_info.def_min_interval = 1,      // Intervalle minimum par défaut
        .u.send_info.def_max_interval = 43200,      // Intervalle maximum par défaut
        .u.send_info.delta.u16 = 400,            // Seuil de changement pour déclencher un rapport (400mm = 40cm)
    };
    
    // Target 3 X coordinate
    esp_zb_zcl_reporting_info_t target3_x_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET3_X_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,          // Intervalle minimum en secondes
        .u.send_info.max_interval = 43200,          // Intervalle maximum en secondes
        .u.send_info.def_min_interval = 1,      // Intervalle minimum par défaut
        .u.send_info.def_max_interval = 43200,      // Intervalle maximum par défaut
        .u.send_info.delta.s16 = 400,            // Seuil de changement pour déclencher un rapport (400mm = 40cm)
    };
    
    // Target 3 Y coordinate
    esp_zb_zcl_reporting_info_t target3_y_reporting_info = {
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
        .ep = HA_ESP_LIGHT_ENDPOINT,
        .cluster_id = ESP_ZB_LD2450_CUSTOM_CLUSTER_ID,
        .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        .attr_id = ESP_ZB_LD2450_ATTR_TARGET3_Y_ID,
        .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .u.send_info.min_interval = 1,          // Intervalle minimum en secondes
        .u.send_info.max_interval = 43200,          // Intervalle maximum en secondes
        .u.send_info.def_min_interval = 1,      // Intervalle minimum par défaut
        .u.send_info.def_max_interval = 43200,      // Intervalle maximum par défaut
        .u.send_info.delta.s16 = 400,            // Seuil de changement pour déclencher un rapport (400mm = 40cm)
    };
    
    ESP_LOGI(TAG, "Configuring automatic reporting for LD2450 target 3 attributes");
    esp_zb_zcl_update_reporting_info(&target3_presence_reporting_info);
    esp_zb_zcl_update_reporting_info(&target3_distance_reporting_info);
    esp_zb_zcl_update_reporting_info(&target3_x_reporting_info);
    esp_zb_zcl_update_reporting_info(&target3_y_reporting_info);
    
    /* Start Zigbee stack */
    ESP_LOGI(TAG, "Starting Zigbee stack...");
    ESP_ERROR_CHECK(esp_zb_start(false));
    //esp_zb_stack_main_loop_iteration();
    esp_zb_main_loop_iteration();

    while(1) {
        vTaskDelay(pdMS_TO_TICKS(100)); // Attente d'1 seconde
    }
}

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

static esp_err_t deferred_driver_init(void)
{
    light_driver_init(LIGHT_DEFAULT_OFF);
    return ESP_OK;
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool light_state = 0;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", 
             message->info.dst_endpoint, message->info.cluster, message->attribute.id, message->attribute.data.size);

    if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
            light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
            ESP_LOGI(TAG, "Light sets to %s", light_state ? "On" : "Off");
            light_driver_set_power(light_state);
        }
    }
    return ret;
}

void app_main(void)
{
    esp_log_level_set("ld2450", ESP_LOG_DEBUG);  // Définir le niveau de log à DEBUG pour le composant ld2450
    
    esp_err_t ret;

    // Initialize NVS first
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create mutex for shared data
    shared_sensor_data.mutex = xSemaphoreCreateMutex();
    if (shared_sensor_data.mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    // Create Zigbee task
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
