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

#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Default I2C address for BH1750 (ADDR pin to GND)
#define BH1750_I2C_ADDRESS_LO 0x23
// Alternative address (ADDR pin to VCC)
#define BH1750_I2C_ADDRESS_HI 0x5C

// BH1750 Commands
#define BH1750_CMD_POWER_DOWN    0x00
#define BH1750_CMD_POWER_ON      0x01
#define BH1750_CMD_RESET         0x07
#define BH1750_CMD_CONT_HRES     0x10  // Continuous high resolution measurement (1 lx)
#define BH1750_CMD_CONT_HRES2    0x11  // Continuous high resolution measurement 2 (0.5 lx)
#define BH1750_CMD_CONT_LRES     0x13  // Continuous low resolution measurement (4 lx)
#define BH1750_CMD_ONETIME_HRES  0x20  // One-time high resolution measurement
#define BH1750_CMD_ONETIME_HRES2 0x21  // One-time high resolution measurement 2
#define BH1750_CMD_ONETIME_LRES  0x23  // One-time low resolution measurement

// BH1750 configuration structure
typedef struct {
    i2c_port_t i2c_port;     // I2C port to use
    uint8_t i2c_address;     // Sensor I2C address
    gpio_num_t sda_pin;      // SDA pin
    gpio_num_t scl_pin;      // SCL pin
    uint32_t timeout_ms;     // Timeout for I2C operations
} bh1750_config_t;

// BH1750 data structure
typedef struct {
    uint16_t light_level;    // Light level in lux
    bool sensor_ok;          // Sensor status
} bh1750_data_t;

/**
 * @brief Detects if a BH1750 sensor is present on the I2C bus
 * 
 * @param i2c_port I2C port to use
 * @param i2c_address Sensor I2C address
 * @return esp_err_t ESP_OK if the sensor is detected, otherwise an error
 */
esp_err_t bh1750_detect(i2c_port_t i2c_port, uint8_t i2c_address);

/**
 * @brief Initializes the BH1750 sensor
 * @param config Sensor configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t bh1750_init(const bh1750_config_t *config);

/**
 * @brief Configures the BH1750 measurement mode
 * @param mode Measurement mode (see BH1750_CMD_*)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t bh1750_set_mode(uint8_t mode);

/**
 * @brief Reads the light value from the sensor
 * @param data Structure to store the read data
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t bh1750_read_data(bh1750_data_t *data);

#ifdef __cplusplus
}
#endif
