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
 * along with this program.   If not, see <https://www.gnu.org/licenses/>.
 */

 #pragma once

#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Structure to store target data
typedef struct {
    int16_t x;          // X position in mm (highest bit: 1=positive, 0=negative)
    int16_t y;          // Y position in mm (highest bit: 1=positive, 0=negative)
    int16_t speed;      // Speed in cm/s (highest bit: 1=positive, 0=negative)
    uint16_t resolution; // Distance resolution in mm
} ld2450_target_t;

// Main structure for sensor data
typedef struct {
    bool presence_detected;        // True if presence is detected
    uint8_t num_targets;          // Number of detected targets
    ld2450_target_t targets[3];   // The 3 possible targets
} ld2450_data_t;

// Sensor configuration
typedef struct {
    uart_port_t uart_num;  // UART port number
    int tx_pin;           // GPIO for TX
    int rx_pin;           // GPIO for RX
} ld2450_config_t;

/**
 * @brief Initializes the LD2450 sensor
 * 
 * @param config Sensor configuration (UART and GPIO)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ld2450_init(const ld2450_config_t *config);

/**
 * @brief Reads data from the sensor
 * 
 * @param data Pointer to a structure to store the data
 * @return esp_err_t ESP_OK on success
 */
esp_err_t ld2450_read_data(ld2450_data_t *data);

#ifdef __cplusplus
}
#endif
