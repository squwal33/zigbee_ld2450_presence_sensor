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

 #include <string.h>
#include "esp_log.h"
#include "bh1750.h"
#include "driver/i2c.h"
#include "freertos/task.h"

static const char *TAG = "BH1750";

// Static variables to store configuration
static i2c_port_t i2c_port = I2C_NUM_MAX;
static uint8_t device_address = 0;
static uint32_t i2c_timeout = 0;

// Utility function to display I2C errors
static void log_i2c_error(const char* operation, esp_err_t err) {
    if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "%s failed: timeout", operation);
    } else if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "%s failed: device not found", operation);
    } else if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "%s failed: invalid state", operation);
    } else if (err == ESP_ERR_INVALID_ARG) {
        ESP_LOGE(TAG, "%s failed: invalid argument", operation);
    } else {
        ESP_LOGE(TAG, "%s failed: %s (0x%x)", operation, esp_err_to_name(err), err);
    }
}

esp_err_t bh1750_detect(i2c_port_t i2c_port, uint8_t i2c_address) {
    ESP_LOGI(TAG, "Detecting BH1750 sensor on I2C address 0x%02X...", i2c_address);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C command");
        return ESP_ERR_NO_MEM;
    }

    // Try sending a reset command
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, BH1750_CMD_POWER_ON, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "BH1750 sensor found!");
    } else {
        log_i2c_error("Sensor detection", ret);
    }

    return ret;
}

esp_err_t bh1750_init(const bh1750_config_t *config) {
    ESP_LOGI(TAG, "Initializing BH1750 sensor on I2C address 0x%02X", config->i2c_address);
    
    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid configuration (NULL)");
        return ESP_ERR_INVALID_ARG;
    }

    // Save parameters
    i2c_port = config->i2c_port;
    device_address = config->i2c_address;
    i2c_timeout = config->timeout_ms / portTICK_PERIOD_MS;

    // Check if the sensor is present
    esp_err_t ret = bh1750_detect(i2c_port, device_address);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BH1750 sensor not found during initialization");
        return ret;
    }

    // Power on the sensor
    ret = bh1750_set_mode(BH1750_CMD_POWER_ON);
    if (ret != ESP_OK) {
        log_i2c_error("Power on", ret);
        return ret;
    }

    // Wait for the sensor to be ready
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configure in continuous high resolution mode
    ret = bh1750_set_mode(BH1750_CMD_CONT_HRES);
    if (ret != ESP_OK) {
        log_i2c_error("Set mode", ret);
        return ret;
    }

    ESP_LOGI(TAG, "BH1750 initialized successfully");
    return ESP_OK;
}

esp_err_t bh1750_set_mode(uint8_t mode) {
    if (i2c_port >= I2C_NUM_MAX) {
        ESP_LOGE(TAG, "I2C port not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C command");
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, mode, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, i2c_timeout);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        log_i2c_error("Set mode", ret);
    } else {
        ESP_LOGD(TAG, "Set mode to 0x%02x", mode);
    }

    return ret;
}

esp_err_t bh1750_read_data(bh1750_data_t *data) {
    if (data == NULL) {
        ESP_LOGE(TAG, "Invalid data pointer (NULL)");
        return ESP_ERR_INVALID_ARG;
    }

    if (i2c_port >= I2C_NUM_MAX) {
        ESP_LOGE(TAG, "I2C port not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t raw_data[2];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C command");
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, raw_data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, i2c_timeout);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        // Convert raw data to lux
        // Format: MSB then LSB
        data->light_level = ((raw_data[0] << 8) | raw_data[1]) / 1.2;  // Division by 1.2 according to datasheet
        data->sensor_ok = true;
        /*ESP_LOGI(TAG, "Raw data: 0x%02X%02X, Light level: %d lux", raw_data[0], raw_data[1], data->light_level);*/
    } else {
        data->light_level = 0;
        data->sensor_ok = false;
        log_i2c_error("Read data", ret);
    }

    return ret;
}
