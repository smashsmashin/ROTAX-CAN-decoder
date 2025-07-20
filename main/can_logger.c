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
    // For ESP32-C3, direct integer values for TSEG1, TSEG2, SJW are expected
    // For 500kbps: TQ = 0.2us. With APB_CLK=80MHz, brp should be 16 (TQ = brp / APB_CLK for ESP32-C3 HAL).
    // Total TQ per bit = 1 (sync) + tseg_1 (6) + tseg_2 (3) = 10 TQ.
    // Bit time = 10 TQ * 0.2us/TQ = 2us => 1 / 2us = 500kbps.
    twai_timing_config_t t_config = {
        .brp = 16, // Baudrate prescaler (corrected from 8 for 500kbps)
        .tseg_1 = 6, // Timing segment 1
        .tseg_2 = 3, // Timing segment 2
        .sjw = 2,    // Synchronization jump width
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

    uint32_t last_led_toggle_time_ms = 0;
    bool heartbeat_led_state = false;
    uint32_t last_alive_message_time_ms = 0;
    const uint32_t alive_message_interval_ms = 10000; // 10 seconds
    const uint32_t led_blink_interval_ms = 500;     // 0.5 seconds for ~1Hz blink

    // Main loop to receive and log TWAI messages
    while (1) {
        uint32_t current_time_ms = esp_log_timestamp(); // Or use esp_timer_get_time() / 1000 for FreeRTOS ticks

        twai_message_t message; // Changed from can_message_t
        esp_err_t ret = twai_receive(&message, pdMS_TO_TICKS(100)); // Wait for 100ms, changed from can_receive
        twai_status_info_t status_info; // Changed from can_status_info_t
        esp_err_t status_err = twai_get_status_info(&status_info); // Changed from can_get_status_info

        bool bus_error_or_recovering = false;

        if (status_err == ESP_OK && status_info.state == TWAI_STATE_BUS_OFF) {
            bus_error_or_recovering = true;
            ESP_LOGW(TAG, "TWAI Bus Off detected. Attempting recovery...");
            gpio_set_level(STATUS_LED_GPIO, 1); // Turn LED ON for bus-off
            esp_err_t recovery_err = twai_initiate_recovery(); // Corrected function call
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
                ESP_LOGE(TAG, "TWAI bus recovery initiation failed. Error: %s", esp_err_to_name(recovery_err));
                // LED remains ON (bus_error_or_recovering is true, so LED will be set to 1)
            }
        } else if (status_err == ESP_OK && status_info.state == TWAI_STATE_RECOVERING) {
            bus_error_or_recovering = true;
            ESP_LOGI(TAG, "TWAI bus is in RECOVERING state.");
            // LED will be set to 1 due to bus_error_or_recovering
        } else if (status_err != ESP_OK) {
            // Failed to get bus status, assume error for LED
            bus_error_or_recovering = true;
            ESP_LOGE(TAG, "Failed to get TWAI status. Error: %s", esp_err_to_name(status_err));
        }


        if (ret == ESP_OK) {
            // Message received
            if (!bus_error_or_recovering && status_info.state == TWAI_STATE_RUNNING) {
                gpio_set_level(STATUS_LED_GPIO, 0); // Solid OFF when actively receiving and bus OK
            }
            // Log received message to USB serial
            printf("TWAI RX: ID=0x%03lX DLC=%d Data=", message.identifier, message.data_length_code);
            for (int i = 0; i < message.data_length_code; i++) {
                printf("0x%02X ", message.data[i]);
            }
            if (message.flags & TWAI_MSG_FLAG_EXTD) {
                printf("(EXTD) ");
            }
            if (message.flags & TWAI_MSG_FLAG_RTR) {
                printf("(RTR) ");
            }
            printf("\n");

        } else if (ret == ESP_ERR_TIMEOUT) {
            // No message received within timeout
            // Heartbeat LED and alive message logic will apply here if bus is RUNNING
        } else {
            // Other error from twai_receive
            ESP_LOGE(TAG, "Failed to receive TWAI message, error: %s", esp_err_to_name(ret));
            // The check for BUS_OFF at the start of the loop should also cover this scenario if it leads to bus off.
            // If not bus_off, but some other receive error, bus_error_or_recovering might still be false.
            // Consider if LED should indicate this type of error too. For now, it doesn't explicitly.
        }

        // Manage LED based on overall state
        if (bus_error_or_recovering) {
            gpio_set_level(STATUS_LED_GPIO, 1); // Solid ON for BUS_OFF or RECOVERING
            heartbeat_led_state = false; // Reset heartbeat state if error occurs
        } else {
            // Bus is RUNNING (or status couldn't be read but no specific error state found)
            // Implement heartbeat blink if no messages were received in this cycle (ret == ESP_ERR_TIMEOUT)
            // or if we decide to blink even with messages (current logic: solid off with messages).
            // For now, blink only on timeout and bus RUNNING.
            if (status_err == ESP_OK && status_info.state == TWAI_STATE_RUNNING) {
                if (current_time_ms - last_led_toggle_time_ms >= led_blink_interval_ms) {
                    heartbeat_led_state = !heartbeat_led_state;
                    gpio_set_level(STATUS_LED_GPIO, heartbeat_led_state);
                    last_led_toggle_time_ms = current_time_ms;
                }
            } else if (status_err != ESP_OK) {
                // If status read failed, and not already handled as bus_error_or_recovering
                // default to LED ON to indicate some unknown issue.
                 gpio_set_level(STATUS_LED_GPIO, 1);
            }
            // If ret == ESP_OK (message received), LED is set to 0 above.
            // This means heartbeat blinking is overridden by message reception.
        }

        // Periodic "System alive" message
        if (status_err == ESP_OK && status_info.state == TWAI_STATE_RUNNING) {
            if (current_time_ms - last_alive_message_time_ms >= alive_message_interval_ms) {
                // It's time to print the alive message
                // Optionally, only print if no CAN messages were received in this specific cycle,
                // or just print regardless as a general heartbeat.
                // For "running without can" check, printing when idle is good.
                if (ret == ESP_ERR_TIMEOUT) { // Only print if no CAN message was received in the last 100ms check
                    printf("[%lu ms] System alive. TWAI Bus: RUNNING. No new CAN messages in last check cycle.\n", current_time_ms);
                } else if (ret == ESP_OK) {
                    // If messages are flowing, we could print a different alive message or skip.
                    // For now, let's also indicate alive even with messages, but slightly different.
                     printf("[%lu ms] System alive. TWAI Bus: RUNNING. Actively processing CAN messages.\n", current_time_ms);
                }
                last_alive_message_time_ms = current_time_ms;
            }
        }


        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
