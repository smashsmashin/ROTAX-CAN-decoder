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
#define OUT_1_GPIO GPIO_NUM_0
#define OUT_2_GPIO GPIO_NUM_1

// Function to initialize LED
static void configure_led(void)
{
    gpio_reset_pin(STATUS_LED_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(STATUS_LED_GPIO, GPIO_MODE_OUTPUT);
}

// Function to initialize Outputs
static void configure_outputs(void)
{
    gpio_reset_pin(OUT_1_GPIO);
    gpio_reset_pin(OUT_2_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(OUT_1_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(OUT_2_GPIO, GPIO_MODE_OUTPUT);

    for(int i=0; i<2; i++) {
        gpio_set_level(OUT_1_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(250));
        gpio_set_level(OUT_1_GPIO, 0);
        gpio_set_level(OUT_2_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(250));
        gpio_set_level(OUT_2_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void signal_failure() {
    // Rapidly blink LED on critical failure
    while(1) {
        gpio_set_level(OUT_1_GPIO, !gpio_get_level(OUT_1_GPIO));
        gpio_set_level(STATUS_LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(OUT_2_GPIO, !gpio_get_level(OUT_2_GPIO));
        gpio_set_level(STATUS_LED_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void)
{
    configure_led();
    configure_outputs();
    gpio_set_level(STATUS_LED_GPIO, 0); // Start with LED OFF

    // Initialize general purpose configuration for TWAI
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
    // Initialize timing configuration for 125kbps
    // APB CLK for ESP32-C3 is typically 80MHz.
    // Bit time for 125kbps = 1 / 125000 = 8us.
    // We need (1 + tseg_1 + tseg_2) * (brp / 80MHz) = 8us.
    // Let's choose a prescaler (brp). A larger prescaler allows for more TQ per bit, giving more flexibility.
    // Let brp = 32. Then tq = 32 / 80MHz = 0.4us.
    // Number of TQ per bit = 8us / 0.4us = 20.
    // So, 1 + tseg_1 + tseg_2 = 20. Let tseg_1 = 15, tseg_2 = 4.
    // This gives a sample point of (1+15)/20 = 80%, which is a common value.
    // SJW is often set to 3 or 4.
    twai_timing_config_t t_config = {
        .brp = 32, // Baudrate prescaler
        .tseg_1 = 15, // Timing segment 1
        .tseg_2 = 4, // Timing segment 2
        .sjw = 3,    // Synchronization jump width
        .triple_sampling = false       // Disable triple sampling
    };

    // Initialize filter configuration (accept all messages)
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
    ESP_LOGI(TAG, "Installing TWAI driver...");
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver.");
        signal_failure();
    }
    ESP_LOGI(TAG, "TWAI driver installed.");

    // Start TWAI driver
    ESP_LOGI(TAG, "Starting TWAI driver...");
    if (twai_start() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver.");
        signal_failure();
    }
    ESP_LOGI(TAG, "TWAI driver started.");
    gpio_set_level(STATUS_LED_GPIO, 0); // LED OFF, TWAI initialized successfully

    ESP_LOGI(TAG, "TWAI Logger Initialized. Waiting for messages...");

    bool packet_received = false;
    uint32_t led_on_time_ms = 0;
    const uint32_t led_on_interval_ms = 100;     // 0.5 seconds for ~1Hz blink

    uint32_t last_alive_message_time_ms = 0;
    const uint32_t alive_message_interval_ms = 10000; // 10 seconds

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

            // Check for specific CAN IDs
            switch (message.identifier) {
                case 556: // Engine 1 Status (Line A)
                case 557: // Engine 2 Status (Line A)
                case 620: // Engine 1 Status (Line B)
                case 621: // Engine 2 Status (Line B)
                    if (message.data_length_code == 8 && message.data[1] == 5 /*BLONG*/) {
                        packet_received = true;
                        led_on_time_ms = current_time_ms;
                        gpio_set_level(STATUS_LED_GPIO, 1);

                        uint8_t status_byte_4 = message.data[7];
                        // Bit 2 of byte 7 indicates the mode
                        // 0: Power Mode, 1: Economy Mode
                        if (status_byte_4 & 0x02) { // Economy Mode
                            ESP_LOGI(TAG, "Economy Mode detected");
                            gpio_set_level(OUT_1_GPIO, 1);
                            gpio_set_level(OUT_2_GPIO, 0);
                        } else { // Power Mode
                            ESP_LOGI(TAG, "Power Mode detected");
                            gpio_set_level(OUT_1_GPIO, 0);
                            gpio_set_level(OUT_2_GPIO, 1);
                        }
                    }
                    break;
                default:
                    // Not an engine status message
                    break;
            }

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
        if (bus_error_or_recovering || status_err != ESP_OK) {
            gpio_set_level(STATUS_LED_GPIO, 1); // Solid ON for BUS_OFF or RECOVERING
            packet_received = false; // Reset heartbeat state if error occurs
        } else if(packet_received) {
            if (current_time_ms - led_on_time_ms >= led_on_interval_ms) {
                packet_received = false;
                gpio_set_level(STATUS_LED_GPIO, 0);
            }
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
