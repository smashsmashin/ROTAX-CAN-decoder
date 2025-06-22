#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/can.h"
#include "esp_log.h"

// Define CAN pins based on schematic
#define CAN_TX_PIN GPIO_NUM_21
#define CAN_RX_PIN GPIO_NUM_20

// Tag for logging
static const char *TAG = "CAN_LOGGER";

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

    // Initialize general purpose configuration
    can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, CAN_MODE_NORMAL);
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
    can_timing_config_t t_config = {
        .brp = 8,
        .tseg1 = CAN_TIMING_TS1_6TQ, // Using common values
        .tseg2 = CAN_TIMING_TS2_3TQ,
        .sjw = CAN_TIMING_SJW_2TQ,   // Synchronization Jump Width
        .triple_sampling = false
    };

    // Initialize filter configuration (accept all messages)
    can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

    // Install CAN driver
    ESP_LOGI(TAG, "Installing CAN driver...");
    if (can_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install CAN driver.");
        // Rapidly blink LED on critical failure
        while(1) {
            gpio_set_level(STATUS_LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(STATUS_LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    ESP_LOGI(TAG, "CAN driver installed.");

    // Start CAN driver
    ESP_LOGI(TAG, "Starting CAN driver...");
    if (can_start() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start CAN driver.");
        // Rapidly blink LED on critical failure
        while(1) {
            gpio_set_level(STATUS_LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(STATUS_LED_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    ESP_LOGI(TAG, "CAN driver started.");
    gpio_set_level(STATUS_LED_GPIO, 0); // LED OFF, CAN initialized successfully

    ESP_LOGI(TAG, "CAN Logger Initialized. Waiting for messages...");

    // Main loop to receive and log CAN messages
    while (1) {
        can_message_t message;
        esp_err_t ret = can_receive(&message, pdMS_TO_TICKS(100)); // Wait for 100ms
        can_status_info_t status_info;
        esp_err_t status_err = can_get_status_info(&status_info);

        if (status_err == ESP_OK && status_info.state == CAN_STATE_BUS_OFF) {
            ESP_LOGW(TAG, "CAN Bus Off detected. Attempting recovery...");
            gpio_set_level(STATUS_LED_GPIO, 1); // Turn LED ON for bus-off
            esp_err_t recovery_err = can_recover_from_bus_off(pdMS_TO_TICKS(1000));
            if (recovery_err == ESP_OK) {
                ESP_LOGI(TAG, "CAN bus recovery initiated.");
                // Check status again after attempting recovery
                can_get_status_info(&status_info);
                if (status_info.state == CAN_STATE_RUNNING) {
                    ESP_LOGI(TAG, "CAN bus recovered to RUNNING state.");
                    gpio_set_level(STATUS_LED_GPIO, 0); // Turn LED OFF
                } else {
                    ESP_LOGW(TAG, "CAN bus still not RUNNING after recovery attempt. State: %d", status_info.state);
                    // LED remains ON
                }
            } else {
                ESP_LOGE(TAG, "CAN bus recovery initiation failed.");
                // LED remains ON
            }
        } else if (status_err == ESP_OK && status_info.state == CAN_STATE_RECOVERING) {
            ESP_LOGI(TAG, "CAN bus is in RECOVERING state.");
            gpio_set_level(STATUS_LED_GPIO, 1); // LED ON during recovery
        }


        if (ret == ESP_OK) {
            // Log received message to USB serial
            printf("CAN RX: ID=0x%03lX DLC=%d Data=", message.identifier, message.data_length_code);
            for (int i = 0; i < message.data_length_code; i++) {
                printf("0x%02X ", message.data[i]);
            }
            if (message.flags & CAN_MSG_FLAG_EXTD) {
                printf("(EXTD) ");
            }
            if (message.flags & CAN_MSG_FLAG_RTR) {
                printf("(RTR) ");
            }
            printf("\n");

            // If we are receiving messages, bus should be running, turn LED off if it was on.
            if (status_err == ESP_OK && status_info.state == CAN_STATE_RUNNING) {
                gpio_set_level(STATUS_LED_GPIO, 0);
            }

        } else if (ret == ESP_ERR_TIMEOUT) {
            // No message received. If bus is running, LED should be off.
            // If bus was off and recovered to running but no messages yet, LED is handled above.
            if (status_err == ESP_OK && status_info.state == CAN_STATE_RUNNING) {
                gpio_set_level(STATUS_LED_GPIO, 0);
            }
            // If timeout occurs and bus is found to be OFF, the check at the beginning of the loop will handle it.
        } else { // Other errors from can_receive
            ESP_LOGE(TAG, "Failed to receive CAN message, error: %s", esp_err_to_name(ret));
            // The check for BUS_OFF at the start of the loop should also cover this scenario if it leads to bus off.
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to yield to other tasks
    }
}
