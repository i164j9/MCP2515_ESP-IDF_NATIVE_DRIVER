// demo.c - Minimal example for MCP2515 driver with ESP-IDF, Bluetooth SPP, and CAN task management
#include "include/mcp2515.h"
#include "include/can_bridge.h"
#include "include/spp_bridge.h"
#include "bluetooth_manager.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define TAG "DEMO"

void app_main(void) {
    // Initialize NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    esp_log_level_set("MCP2515", ESP_LOG_INFO);

    // Configure MCP2515 INT and CS pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MCP2515_INT_PIN) | (1ULL << MCP2515_CS_PIN),
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_direction(MCP2515_CS_PIN, GPIO_MODE_OUTPUT);

    // Initialize Bluetooth
    ESP_ERROR_CHECK(bluetooth_manager_init());
    ESP_LOGI(TAG, "Bluetooth manager initialized");

    // Initialize MCP2515
    static mcp2515_t mcp2515_dev;
    ESP_ERROR_CHECK(mcp2515_init(&mcp2515_dev, VSPI_HOST, MCP2515_CS_PIN));
    ESP_LOGI(TAG, "MCP2515 initialized");

    // Set CAN bitrate to 500k (16MHz clock)
    ESP_ERROR_CHECK(mcp2515_config_bitrate(&mcp2515_dev, 500000, 16));
    ESP_LOGI(TAG, "MCP2515 bitrate set to 500k");

    // (Optional) Enable loopback mode for testing
    // ESP_ERROR_CHECK(mcp2515_set_loopback(&mcp2515_dev));
    // ESP_LOGI(TAG, "MCP2515 set to loopback mode");

    // (Optional) Set RX filter/mask to accept only 0x123
    // uint8_t mask[4] = {0xFF, 0xE0, 0, 0};
    // uint8_t filter[4] = {0x24, 0x60, 0, 0};
    // ESP_ERROR_CHECK(mcp2515_set_filter_mask(&mcp2515_dev, mask, filter));
    // ESP_LOGI(TAG, "MCP2515 RXB0 filter set to 0x123");

    // Initialize CAN bridge
    can_bridge_init(3072, configMAX_PRIORITIES - 1, 0);
    ESP_LOGI(TAG, "CAN bridge initialized");

    // Start CAN TX task
    xTaskCreatePinnedToCore(can_tx_task, "can_tx_task", 3072, &mcp2515_dev, configMAX_PRIORITIES - 1, NULL, 0);
    ESP_LOGI(TAG, "CAN TX task started");

    // Start Bluetooth SPP bridge
    spp_bridge_init(3072, configMAX_PRIORITIES - 2, 3072, configMAX_PRIORITIES - 3, 1);
    ESP_LOGI(TAG, "Bluetooth SPP CAN bridge started");

    // Demo: Send a standard CAN frame
    CAN_frame_t frame = {
        .MsgID = 0x123,
        .FMT = CAN_frame_std,
        .RTR = CAN_no_RTR,
        .DLC = 8,
        .data = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88}
    };
    mcp2515_write_frame(&mcp2515_dev, &frame);
    ESP_LOGI(TAG, "Demo CAN frame sent");

    // Main loop
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
