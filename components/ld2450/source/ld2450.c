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
#include <stdbool.h>
#include "esp_log.h"
#include "driver/uart.h"
#include "ld2450.h"
#include <math.h>

static const char *TAG = "LD2450";

#define LD2450_UART_BUF_SIZE (2048)
#define LD2450_FRAME_HEADER1 (0xAA)
#define LD2450_FRAME_HEADER2 (0xFF)
#define LD2450_FRAME_HEADER3 (0x03)
#define LD2450_FRAME_HEADER4 (0x00)
#define LD2450_FRAME_FOOTER1 (0x55)
#define LD2450_FRAME_FOOTER2 (0xCC)
#define LD2450_FRAME_LENGTH (30)
#define LD2450_READ_TIMEOUT_MS (1000)
#define MAX_TARGETS 3

static uart_port_t uart_num;

// Forward declaration of internal functions
static esp_err_t wait_for_frame_header(uint8_t *buffer);
static esp_err_t ld2450_parse_frame(const uint8_t *frame_data, size_t frame_len, ld2450_data_t *data);

esp_err_t ld2450_init(const ld2450_config_t *config) {
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uart_num = config->uart_num;

    // Configuration UART
    uart_config_t uart_config = {
        .baud_rate = 256000,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Désinstaller le driver UART s'il était déjà installé
    uart_driver_delete(uart_num);

    esp_err_t ret = uart_driver_install(uart_num, LD2450_UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install uart driver: %d", ret);
        return ret;
    }

    ret = uart_param_config(uart_num, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure uart parameters: %d", ret);
        return ret;
    }

    ret = uart_set_pin(uart_num, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set uart pins: %d", ret);
        return ret;
    }

    // Vider le buffer UART au démarrage
    uart_flush(uart_num);
    
    ESP_LOGD(TAG, "LD2450 initialized successfully");
    return ESP_OK;
}

static esp_err_t wait_for_frame_header(uint8_t *buffer) {
    if (buffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t bytes_read = 0;
    uint8_t byte;
    int state = 0;
    int attempts = 0;
    int timeouts = 0;
    const int max_attempts = 100; // Limiter le nombre d'octets à lire pour éviter de bloquer trop longtemps
    const int max_timeouts = 5;   // Nombre maximum de timeouts consécutifs avant d'abandonner

    // Vérifier combien d'octets sont disponibles dans le buffer UART
    size_t available_bytes;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, &available_bytes));
    //ESP_LOGI(TAG, "Bytes available in UART buffer: %d", available_bytes);

    // Si le buffer est vide, retourner immédiatement un timeout
    if (available_bytes == 0) {
        return ESP_ERR_TIMEOUT;
    }

    // Attendre l'en-tête de trame (AA FF 03 00)
    while (state < 4 && attempts < max_attempts) {
        attempts++;
        
        // Lire un octet avec un timeout court
        int result = uart_read_bytes(uart_num, &byte, 1, pdMS_TO_TICKS(5));
        
        if (result != 1) {
            timeouts++;
            if (timeouts >= max_timeouts) {
                ESP_LOGD(TAG, "Too many consecutive timeouts (%d), giving up", timeouts);
                return ESP_ERR_TIMEOUT;
            }
            continue; // Essayer de lire le prochain octet
        }
        
        // Réinitialiser le compteur de timeouts car nous avons lu un octet avec succès
        timeouts = 0;

        // Traiter l'octet lu selon l'état actuel
        switch (state) {
            case 0:
                if (byte == LD2450_FRAME_HEADER1) {
                    buffer[bytes_read++] = byte;
                    state++;
                    ESP_LOGD(TAG, "Found header byte 1: 0x%02X", byte);
                }
                break;
            case 1:
                if (byte == LD2450_FRAME_HEADER2) {
                    buffer[bytes_read++] = byte;
                    state++;
                    ESP_LOGD(TAG, "Found header byte 2: 0x%02X", byte);
                } else {
                    // Si ce n'est pas le bon octet, vérifier si c'est le début d'une nouvelle séquence
                    if (byte == LD2450_FRAME_HEADER1) {
                        buffer[0] = byte;
                        bytes_read = 1;
                        state = 1;
                        ESP_LOGD(TAG, "Restarting with new header byte 1: 0x%02X", byte);
                    } else {
                        state = 0;
                        bytes_read = 0; // Réinitialiser le buffer
                    }
                }
                break;
            case 2:
                if (byte == LD2450_FRAME_HEADER3) {
                    buffer[bytes_read++] = byte;
                    state++;
                    ESP_LOGD(TAG, "Found header byte 3: 0x%02X", byte);
                } else {
                    // Si ce n'est pas le bon octet, vérifier si c'est le début d'une nouvelle séquence
                    if (byte == LD2450_FRAME_HEADER1) {
                        buffer[0] = byte;
                        bytes_read = 1;
                        state = 1;
                        ESP_LOGD(TAG, "Restarting with new header byte 1: 0x%02X", byte);
                    } else {
                        state = 0;
                        bytes_read = 0; // Réinitialiser le buffer
                    }
                }
                break;
            case 3:
                if (byte == LD2450_FRAME_HEADER4) {
                    buffer[bytes_read++] = byte;
                    state++;
                    ESP_LOGD(TAG, "Found header byte 4: 0x%02X", byte);
                } else {
                    // Si ce n'est pas le bon octet, vérifier si c'est le début d'une nouvelle séquence
                    if (byte == LD2450_FRAME_HEADER1) {
                        buffer[0] = byte;
                        bytes_read = 1;
                        state = 1;
                        ESP_LOGD(TAG, "Restarting with new header byte 1: 0x%02X", byte);
                    } else {
                        state = 0;
                        bytes_read = 0; // Réinitialiser le buffer
                    }
                }
                break;
        }
    }

    if (state < 4) {
        ESP_LOGW(TAG, "Failed to find complete frame header after %d attempts", attempts);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGD(TAG, "Frame header found");
    return ESP_OK;
}

esp_err_t ld2450_read_data(ld2450_data_t *data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if data is available on UART
    size_t available_bytes;
    uart_get_buffered_data_len(uart_num, &available_bytes);
    //ESP_LOGI(TAG, "Bytes available in UART buffer: %d", available_bytes);
    
    // If there's not enough data for at least one complete frame, return
    if (available_bytes < LD2450_FRAME_LENGTH) {
        ESP_LOGW(TAG, "Not enough data for a complete frame (%d bytes)", available_bytes);
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Calculate how many complete frames might be in the buffer
    size_t potential_frames = available_bytes / LD2450_FRAME_LENGTH;
    ESP_LOGD(TAG, "Potentially %d complete frames in buffer", potential_frames);
    
    // If we have multiple frames, we want to skip to the most recent one
    // We'll read and discard all but the last potential frame
    if (potential_frames > 1) {
        // Calculate bytes to discard (all frames except the last one)
        size_t bytes_to_discard = (potential_frames - 1) * LD2450_FRAME_LENGTH;
        
        // Temporary buffer for discarding bytes
        uint8_t discard_buffer[64];
        size_t bytes_discarded = 0;
        
        // Discard bytes in chunks
        while (bytes_discarded < bytes_to_discard) {
            size_t chunk_size = (bytes_to_discard - bytes_discarded) > sizeof(discard_buffer) ? 
                                sizeof(discard_buffer) : (bytes_to_discard - bytes_discarded);
            
            int len = uart_read_bytes(uart_num, discard_buffer, chunk_size, pdMS_TO_TICKS(LD2450_READ_TIMEOUT_MS));
            if (len < 0) {
                ESP_LOGW(TAG, "Error discarding bytes");
                uart_flush_input(uart_num);
                return ESP_FAIL;
            }
            
            bytes_discarded += len;
        }
        
        ESP_LOGD(TAG, "Discarded %d bytes to reach the most recent frame", bytes_discarded);
    }
    
    // Now we should be positioned at the start of the most recent frame
    // But we still need to find the frame header for proper synchronization
    uint8_t frame_buffer[LD2450_FRAME_LENGTH] = {0};
    
    // Wait for frame header
    esp_err_t ret = wait_for_frame_header(frame_buffer);
    if (ret != ESP_OK) {
        uart_flush_input(uart_num);
        return ret;
    }
    
    // Read the rest of the frame
    int len = uart_read_bytes(uart_num, frame_buffer + 4, LD2450_FRAME_LENGTH - 4, pdMS_TO_TICKS(LD2450_READ_TIMEOUT_MS));
    if (len != LD2450_FRAME_LENGTH - 4) {
        ESP_LOGW(TAG, "Failed to read complete frame, got %d bytes instead of %d", len, LD2450_FRAME_LENGTH - 4);
        uart_flush_input(uart_num);
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Now that we have read the complete frame, we can flush the UART buffer
    // to avoid data accumulation
    uart_flush_input(uart_num);
    
    // Check the footer
    if (frame_buffer[LD2450_FRAME_LENGTH - 2] != LD2450_FRAME_FOOTER1 ||
        frame_buffer[LD2450_FRAME_LENGTH - 1] != LD2450_FRAME_FOOTER2) {
        ESP_LOGW(TAG, "Invalid frame footer: %02X %02X (expected: %02X %02X)", 
                frame_buffer[LD2450_FRAME_LENGTH - 2], frame_buffer[LD2450_FRAME_LENGTH - 1],
                LD2450_FRAME_FOOTER1, LD2450_FRAME_FOOTER2);
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGD(TAG, "Frame received");
    return ld2450_parse_frame(frame_buffer, LD2450_FRAME_LENGTH, data);
}

static esp_err_t ld2450_parse_frame(const uint8_t *frame_data, size_t frame_len, ld2450_data_t *data) {
    if (frame_data == NULL || data == NULL || frame_len != LD2450_FRAME_LENGTH) {
        return ESP_ERR_INVALID_ARG;
    }

    // Display each byte of the frame in hexadecimal
    ESP_LOGD(TAG, "LD2450 frame data:");
    char hex_buffer[LD2450_FRAME_LENGTH * 3 + 1]; // 3 characters per byte (2 for hex + 1 space) + null terminator
    hex_buffer[0] = '\0';
    for (size_t i = 0; i < frame_len; i++) {
        char temp[4];
        snprintf(temp, sizeof(temp), "%02X ", frame_data[i]);
        strcat(hex_buffer, temp);
    }
    ESP_LOGD(TAG, "%s", hex_buffer);

    // Count the number of non-zero bytes in the data part (between header and footer)
    int non_zero_bytes = 0;
    for (size_t i = 4; i < frame_len - 2; i++) {
        if (frame_data[i] != 0) {
            non_zero_bytes++;
        }
    }
    
    // Inform if the frame contains mostly zeros, but continue processing
    if (non_zero_bytes < 3) {
        ESP_LOGD(TAG, "Frame contains mostly zeros (%d non-zero bytes)", non_zero_bytes);
    }
    
    // Reset data
    memset(data, 0, sizeof(ld2450_data_t));
    data->num_targets = 0;
    bool any_target_detected = false;

    // Analyze the 3 potential targets
    for (int i = 0; i < MAX_TARGETS; i++) {
        int offset = 4 + (i * 8);
        
        // Read raw data - Bytes are in little-endian order in the frame
        uint16_t x_raw = frame_data[offset] | (frame_data[offset + 1] << 8);
        uint16_t y_raw = frame_data[offset + 2] | (frame_data[offset + 3] << 8);
        uint16_t speed_raw = frame_data[offset + 4] | (frame_data[offset + 5] << 8);
        uint16_t resolution = frame_data[offset + 6] | (frame_data[offset + 7] << 8);

        // Display raw values for debugging
        ESP_LOGD(TAG, "Target %d raw values: x_raw=0x%04X, y_raw=0x%04X, speed_raw=0x%04X, resolution=0x%04X",
                i+1, x_raw, y_raw, speed_raw, resolution);

        // Convert coordinates according to specification
        // Most significant bit (MSB) at 1 = positive coordinate
        // Most significant bit (MSB) at 0 = negative coordinate
        
        // Convert X (in mm)
        if (x_raw & 0x8000) {
            // MSB = 1, positive value
            data->targets[i].x = (int16_t)(x_raw - 0x8000);
        } else {
            // MSB = 0, negative value
            data->targets[i].x = -(int16_t)x_raw;
        }
        
        // Convert Y (in mm)
        if (y_raw & 0x8000) {
            // MSB = 1, positive value
            data->targets[i].y = (int16_t)(y_raw - 0x8000);
        } else {
            // MSB = 0, negative value
            data->targets[i].y = -(int16_t)y_raw;
        }
        
        // Convert speed (in cm/s)
        if (speed_raw & 0x8000) {
            // MSB = 1, positive value
            data->targets[i].speed = (int16_t)(speed_raw - 0x8000);
        } else {
            // MSB = 0, negative value
            data->targets[i].speed = -(int16_t)speed_raw;
        }
        
        data->targets[i].resolution = resolution;

        // Calculate distance in meters
        float x_meters = data->targets[i].x / 1000.0f;
        float y_meters = data->targets[i].y / 1000.0f;
        float distance_meters = sqrtf(x_meters * x_meters + y_meters * y_meters);
        
        // A target is considered active if at least one of its values is non-zero
        // AND if the distance is realistic (< 10m)
        if ((x_raw != 0 || y_raw != 0 || speed_raw != 0 || resolution != 0) && distance_meters < 10.0f) {
            data->num_targets++;
            any_target_detected = true;
            ESP_LOGD(TAG, "Target %d: x=%d mm, y=%d mm, speed=%d cm/s, distance=%.2f m", 
                    i+1, data->targets[i].x, data->targets[i].y, data->targets[i].speed, distance_meters);
        } else if (x_raw != 0 || y_raw != 0) {
            // If coordinates are not zero but distance is unrealistic
            ESP_LOGW(TAG, "Target %d has unrealistic distance (%.2f m), ignoring", i+1, distance_meters);
        }
    }

    data->presence_detected = any_target_detected;
    
    if (!any_target_detected) {
        ESP_LOGI(TAG, "No targets detected in this frame");
    }
    
    return ESP_OK;
}
