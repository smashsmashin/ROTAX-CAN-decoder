#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/twai.h" // Changed from can.h to twai.h
#include "esp_log.h"

// Define TWAI pins based on schematic
#define TWAI_TX_PIN GPIO_NUM_21 // Renamed from CAN_TX_PIN
#define TWAI_RX_PIN GPIO_NUM_20 // Renamed from CAN_RX_PIN

// Tag for logging
static const char *TAG = "TWAI_LOGGER"; // Renamed from CAN_LOGGER

// Define Status LED Pin
#define STATUS_LED_GPIO GPIO_NUM_2 // Example: GPIO2 for status LED

// Function to initialize LED
static void configure_led(void)
{
    gpio_reset_pin(STATUS_LED_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(STATUS_LED_GPIO, GPIO_MODE_OUTPUT);
}

void app_main(void)
{
    configure_led();
    gpio_set_level(STATUS_LED_GPIO, 0); // Start with LED OFF

    // Initialize general purpose configuration for TWAI
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
    // Initialize timing configuration for 500kbps
    // Common baud rates: 125kbps, 250kbps, 500kbps, 1Mbps
    // TQ = 1 / (APB CLK / Prescaler)
    // APB CLK for ESP32-C3 is typically 80MHz.
    // For 500kbps, with APB_CLK=80MHz, a common setting is prescaler=8, resulting in 10 TQ per bit.
    // Sync Seg=1, BS1=6, BS2=3. Total 1+6+3 = 10 TQ.
    // (1 + BS1 + BS2) * (Prescaler / APB_CLK) = Bit Time
    // (1 + 6 + 3) * (8 / 80MHz) = 10 * 0.1us = 1us => 1Mbps. This is not 500kbps.
    // Let's recalculate for 500kbps.
    // Bit time = 1 / 500000 = 2us.
    // We need (1 + BS1 + BS2) * (Prescaler / 80MHz) = 2us
    // Let Prescaler = 16. Then (1 + BS1 + BS2) * (16 / 80MHz) = 2us
    // (1 + BS1 + BS2) * 0.2us = 2us
    // (1 + BS1 + BS2) = 10.
    // So, Sync=1, BS1=6, BS2=3 would work. Or BS1=5, BS2=4. Let's use ESP-IDF defaults if possible or common values.
    // ESP-IDF defaults often work well. For 500kbps:
    // brp = 8 -> Tbit = (1+tseg1+tseg2)*tq = (1+15+4)* (2*8/80Mhz) = 20 * 0.2us = 4us -> 250kbps. This is not 500kbps.
    // Let's try settings for 500kbps:
    // Prescaler (brp) = 8. tq = 2 * brp / APB_CLK = 16 / 80MHz = 0.2us
    // Number of TQ per bit = 1 / (500kbps * 0.2us) = 1 / (500000 * 0.0000002) = 1 / 0.1 = 10 TQ.
    // So, we need TSEG1 + TSEG2 + 1 = 10.
    // Let TSEG1 = 6, TSEG2 = 3. (Sample point at 1+6/10 = 70%)
    // ESP32-C3 TWAI controller might have slightly different enums or typical values,
    // but the principle for brp, tseg1, tseg2, sjw remains.
    // For ESP-IDF 4.3+ (which uses TWAI), the timing enum values might be slightly different
    // e.g. TWAI_TIMING_CONFIG_500KBITS() can be used if available and suitable.
    // Let's stick to manual calculation first to ensure it's clear.
    // ESP_IDF twai.h uses .brp, .tseg_1, .tseg_2, .sjw, .triple_sampling
    twai_timing_config_t t_config = {
        .brp = 8, // Baudrate prescaler
        .tseg_1 = TWAI_TIMING_TS1_6TQ, // Timing segment 1
        .tseg_2 = TWAI_TIMING_TS2_3TQ, // Timing segment 2
        .sjw = TWAI_TIMING_SJW_2TQ,    // Synchronization jump width
        .triple_sampling = false       // Disable triple sampling
    };

    // Initialize filter configuration (accept all messages)
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    ESP_LOGI(TAG, "Installing TWAI driver...");
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver.");
        // Rapidly blink LED on critical failure
        while(1) {
            gpio_set_level(STATUS_LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(STATUS_LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    ESP_LOGI(TAG, "TWAI driver installed.");

    // Start TWAI driver
    ESP_LOGI(TAG, "Starting TWAI driver...");
    if (twai_start() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver.");
        // Rapidly blink LED on critical failure
        while(1) {
            gpio_set_level(STATUS_LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(STATUS_LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    ESP_LOGI(TAG, "TWAI driver started.");
    gpio_set_level(STATUS_LED_GPIO, 0); // LED OFF, TWAI initialized successfully

    ESP_LOGI(TAG, "TWAI Logger Initialized. Waiting for messages...");

    // Main loop to receive and log TWAI messages
    while (1) {
        twai_message_t message; // Changed from can_message_t
        esp_err_t ret = twai_receive(&message, pdMS_TO_TICKS(100)); // Wait for 100ms, changed from can_receive
        twai_status_info_t status_info; // Changed from can_status_info_t
        esp_err_t status_err = twai_get_status_info(&status_info); // Changed from can_get_status_info

        if (status_err == ESP_OK && status_info.state == TWAI_STATE_BUS_OFF) { // Changed from CAN_STATE_BUS_OFF
            ESP_LOGW(TAG, "TWAI Bus Off detected. Attempting recovery...");
            gpio_set_level(STATUS_LED_GPIO, 1); // Turn LED ON for bus-off
            esp_err_t recovery_err = twai_recover_from_bus_off(pdMS_TO_TICKS(1000)); // Changed from can_recover_from_bus_off
            if (recovery_err == ESP_OK) {
                ESP_LOGI(TAG, "TWAI bus recovery initiated.");
                // Check status again after attempting recovery
                twai_get_status_info(&status_info); // Changed from can_get_status_info
                if (status_info.state == TWAI_STATE_RUNNING) { // Changed from CAN_STATE_RUNNING
                    ESP_LOGI(TAG, "TWAI bus recovered to RUNNING state.");
                    gpio_set_level(STATUS_LED_GPIO, 0); // Turn LED OFF
                } else {
                    ESP_LOGW(TAG, "TWAI bus still not RUNNING after recovery attempt. State: %d", status_info.state);
                    // LED remains ON
                }
            } else {
                ESP_LOGE(TAG, "TWAI bus recovery initiation failed."); // Corrected log
                // LED remains ON
            }
        } else if (status_err == ESP_OK && status_info.state == TWAI_STATE_RECOVERING) { // Corrected constant and log
            ESP_LOGI(TAG, "TWAI bus is in RECOVERING state."); // Corrected log
            gpio_set_level(STATUS_LED_GPIO, 1); // LED ON during recovery
        }


        if (ret == ESP_OK) {
            // Log received message to USB serial
            printf("TWAI RX: ID=0x%03lX DLC=%d Data=", message.identifier, message.data_length_code); // Corrected log prefix
            for (int i = 0; i < message.data_length_code; i++) {
                printf("0x%02X ", message.data[i]);
            }
            if (message.flags & TWAI_MSG_FLAG_EXTD) { // Corrected constant
                printf("(EXTD) ");
            }
            if (message.flags & TWAI_MSG_FLAG_RTR) { // Corrected constant
                printf("(RTR) ");
            }
            printf("\n");

            // If we are receiving messages, bus should be running, turn LED off if it was on.
            if (status_err == ESP_OK && status_info.state == TWAI_STATE_RUNNING) { // Corrected constant
                gpio_set_level(STATUS_LED_GPIO, 0);
            }

        } else if (ret == ESP_ERR_TIMEOUT) {
            // No message received. If bus is running, LED should be off.
            // If bus was off and recovered to running but no messages yet, LED is handled above.
            if (status_err == ESP_OK && status_info.state == TWAI_STATE_RUNNING) { // Corrected constant
                gpio_set_level(STATUS_LED_GPIO, 0);
            }
            // If timeout occurs and bus is found to be OFF, the check at the beginning of the loop will handle it.
        } else { // Other errors from twai_receive
            ESP_LOGE(TAG, "Failed to receive TWAI message, error: %s", esp_err_to_name(ret)); // Corrected log
            // The check for BUS_OFF at the start of the loop should also cover this scenario if it leads to bus off.
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to yield to other tasks
    }
}
