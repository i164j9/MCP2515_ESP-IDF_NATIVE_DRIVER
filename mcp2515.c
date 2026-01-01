#include "mcp2515.h"
#include "esp_log.h"

#define TAG "MCP2515"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdio.h>

static const char *mcp2515_mode_str(uint8_t canstat) {
    switch ((canstat >> 5) & 0x7) {
        case 0: return "Normal";
        case 1: return "Sleep";
        case 2: return "Loopback";
        case 3: return "Listen";
        case 4: return "Config";
        default: return "Unknown";
    }
}
#include "mcp2515.h"
#include "esp_log.h"

#define TAG "MCP2515"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdio.h>

SemaphoreHandle_t mcp2515_spi_mutex = NULL;

static void mcp2515_lock() {
    if (mcp2515_spi_mutex) xSemaphoreTake(mcp2515_spi_mutex, portMAX_DELAY);
}
static void mcp2515_unlock() {
    if (mcp2515_spi_mutex) xSemaphoreGive(mcp2515_spi_mutex);
}

// ESP-IDF native implementation: Write CAN frame to MCP2515
// Assumes frame is valid and dev is initialized
int mcp2515_write_frame(mcp2515_t *dev, const CAN_frame_t *frame) {
    ESP_LOGD(TAG, "CAN TX: MsgID=0x%08X DLC=%d RTR=%d data[0]=0x%02X ...", frame->MsgID, frame->DLC, frame->RTR, frame->data[0]);
    mcp2515_lock();

    // Prepare frame data (SIDH, SIDL, EID8, EID0, DLC, data[0..7])
    uint8_t frame_buf[13] = {0};
    frame_buf[0] = (frame->MsgID >> 3) & 0xFF; // SIDH
    frame_buf[1] = (frame->MsgID & 0x07) << 5; // SIDL
    frame_buf[2] = 0; // EID8
    frame_buf[3] = 0; // EID0
    frame_buf[4] = (frame->DLC & 0x0F) | (frame->RTR ? 0x40 : 0x00); // DLC + RTR
    memcpy(&frame_buf[5], frame->data, (frame->DLC > 8 ? 8 : frame->DLC));

    // 1. Send LOAD_TX_BUF_0 command (0x40)
    uint8_t load_cmd = MCP2515_CMD_LOAD_TX_BUF_0;
    spi_transaction_t t_cmd = {
        .length = 8,
        .tx_buffer = &load_cmd,
        .rx_buffer = NULL,
    };
    esp_err_t ret = spi_device_transmit(dev->spi, &t_cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcp2515_write_frame: LOAD_TX_BUF_0 cmd failed: %d", ret);
        mcp2515_unlock();
        return -1;
    }

    // 2. Send frame data
    spi_transaction_t t_data = {
        .length = (5 + frame->DLC) * 8,
        .tx_buffer = frame_buf,
        .rx_buffer = NULL,
    };
    ret = spi_device_transmit(dev->spi, &t_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcp2515_write_frame: frame data transmit failed: %d", ret);
        mcp2515_unlock();
        return -1;
    }

    // 3. Send RTS command for TX buffer 0
    uint8_t rts_cmd = MCP2515_CMD_RTS_TXB0;
    spi_transaction_t t_rts = {
        .length = 8,
        .tx_buffer = &rts_cmd,
        .rx_buffer = NULL,
        .rxlength = 0,
    };

    ret = spi_device_transmit(dev->spi, &t_rts);
    ESP_LOGI(TAG, "mcp2515_write_frame: RTS command result: %s", (ret == ESP_OK) ? "ESP_OK" : "FAIL");
    mcp2515_unlock();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcp2515_write_frame: RTS failed: %d", ret);
        return -1;
    }

    // --- Diagnostics: Batch read CANINTF, EFLG, TXB0CTRL, CANSTAT, CANCTRL ---
    uint8_t regs[5] = {0};
    {
        uint8_t tx[2] = {MCP2515_CMD_READ, MCP2515_REG_CANINTF};
        uint8_t rx[7] = {0};
        spi_transaction_t t = {
            .length = 8 * 7, // 2 command + 5 data
            .tx_buffer = tx,
            .rx_buffer = rx,
        };
        mcp2515_lock();
        esp_err_t ret_diag = spi_device_transmit(dev->spi, &t);
        mcp2515_unlock();
        if (ret_diag == ESP_OK) {
            memcpy(regs, &rx[2], 5);
        } else {
            ESP_LOGE(TAG, "Batch reg read failed: %d", ret_diag);
        }
    }
    ESP_LOGW(TAG, "TX post-status: CANINTF=0x%02X EFLG=0x%02X TXB0CTRL=0x%02X CANSTAT=0x%02X (%s) CANCTRL=0x%02X", regs[0], regs[1], regs[2], regs[3], mcp2515_mode_str(regs[3]), regs[4]);
    return 0;
}

// ESP-IDF native, thread-safe, modular implementation
// Reads a CAN frame from MCP2515 RX buffer 0
// Returns 0 on success, -1 if no frame or error
int mcp2515_read_frame(mcp2515_t *dev, CAN_frame_t *frame) {
    mcp2515_lock();

    // Read RX status to check for available frame
    uint8_t rx_status = mcp2515_read_status(dev);
    if ((rx_status & 0xC0) == 0) {
        // No message in RXB0 or RXB1
        mcp2515_unlock();
        return -1;
    }

    // Read RX buffer 0 (0x90)
    uint8_t tx_buf[1] = {MCP2515_CMD_READ_RX_BUF_0};
    uint8_t rx_buf[14] = {0};
    spi_transaction_t t = {
        .length = 8 * (1 + 13), // 1 command + 13 bytes
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };
    esp_err_t ret = spi_device_transmit(dev->spi, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "mcp2515_read_frame: spi_device_transmit failed: %d", ret);
        mcp2515_unlock();
        return -1;
    }

    // rx_buf[1..5]: ID, DLC, etc.
    uint8_t *d = &rx_buf[1];
    uint32_t id = (d[0] << 3) | (d[1] >> 5);
    bool ext = false;
    if (d[1] & 0x08) {
        // Extended ID
        id = ((id << 2) | (d[1] & 0x03));
        id = ((id << 8) | d[2]);
        id = ((id << 8) | d[3]);
        ext = true;
    }
    frame->MsgID = id;
    frame->FMT = ext ? CAN_frame_ext : CAN_frame_std;
    frame->RTR = (d[4] & 0x40) ? CAN_RTR : CAN_no_RTR;
    frame->DLC = d[4] & 0x0F;
    for (int i = 0; i < frame->DLC && i < 8; i++) {
        frame->data[i] = d[5 + i];
    }

    // Clear RX interrupt flag (RX0IF)
    mcp2515_bit_modify(dev, MCP2515_REG_CANINTF, 0x01, 0x00);

    mcp2515_unlock();
    return 0;
}

// Minimal MCP2515 driver for ESP-IDF (no Arduino)

esp_err_t mcp2515_init(mcp2515_t *dev, spi_host_device_t host, int cs_gpio) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = 23,
        .miso_io_num = 19,
        .sclk_io_num = 18,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 250*1000, // 250 kHz (lowered for signal integrity)
        .mode = 0,
        .spics_io_num = cs_gpio,
        .queue_size = 3,
    };
    if (!mcp2515_spi_mutex) mcp2515_spi_mutex = xSemaphoreCreateMutex();
    esp_err_t ret = spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;
    ret = spi_bus_add_device(host, &devcfg, &dev->spi);
    if (ret != ESP_OK) return ret;
    dev->cs_gpio = cs_gpio;
    return mcp2515_reset(dev);
}

esp_err_t mcp2515_reset(mcp2515_t *dev) {
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = (uint8_t[]){MCP2515_CMD_RESET},
    };
    ESP_LOGI(TAG, "mcp2515_reset: spi_device_transmit RESET");
    esp_err_t ret = spi_device_transmit(dev->spi, &t);
    vTaskDelay(pdMS_TO_TICKS(10));
    return ret;
}

uint8_t mcp2515_read_reg(mcp2515_t *dev, uint8_t reg) {
    uint8_t val = 0xFF;
    int retries = 3;
    for (int attempt = 0; attempt < retries; ++attempt) {
        uint8_t tx[2] = {MCP2515_CMD_READ, reg};
        uint8_t rx[2] = {0};
        spi_transaction_t t = {
            .length = 16,
            .tx_buffer = tx,
            .rx_buffer = rx,
        };
        mcp2515_lock();
        ESP_LOGI(TAG, "mcp2515_read_reg: reg=0x%02X (attempt %d)", reg, attempt+1);
        esp_err_t ret = spi_device_transmit(dev->spi, &t);
        mcp2515_unlock();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "spi_device_transmit failed in read_reg: %d", ret);
            val = 0xFF;
        } else {
            val = rx[1];
            ESP_LOGI(TAG, "mcp2515_read_reg: reg=0x%02X val=0x%02X (attempt %d)", reg, val, attempt+1);
            if (val != 0xFF) {
                break;
            }
        }
        // Only delay if not last attempt and strictly needed (commented out for performance)
        // vTaskDelay(pdMS_TO_TICKS(2));
    }
    return val;
}

esp_err_t mcp2515_write_reg(mcp2515_t *dev, uint8_t reg, uint8_t val) {
    uint8_t tx[3] = {MCP2515_CMD_WRITE, reg, val};
    spi_transaction_t t = {
        .length = 24,
        .tx_buffer = tx,
    };
    mcp2515_lock();
    ESP_LOGI(TAG, "mcp2515_write_reg: reg=0x%02X val=0x%02X", reg, val);
    esp_err_t ret = spi_device_transmit(dev->spi, &t);
    mcp2515_unlock();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_device_transmit failed in write_reg: %d", ret);
    }
    return ret;
}

esp_err_t mcp2515_bit_modify(mcp2515_t *dev, uint8_t reg, uint8_t mask, uint8_t data) {
    uint8_t tx[4] = {MCP2515_CMD_BIT_MODIFY, reg, mask, data};
    spi_transaction_t t = {
        .length = 32,
        .tx_buffer = tx,
    };
    mcp2515_lock();
    ESP_LOGI(TAG, "mcp2515_bit_modify: reg=0x%02X mask=0x%02X data=0x%02X", reg, mask, data);
    esp_err_t ret = spi_device_transmit(dev->spi, &t);
    mcp2515_unlock();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_device_transmit failed in bit_modify: %d", ret);
    }
    return ret;
}

uint8_t mcp2515_read_status(mcp2515_t *dev) {
    uint8_t tx[2] = {MCP2515_CMD_READ_STATUS, 0x00};
    uint8_t rx[2] = {0};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    mcp2515_lock();
    ESP_LOGI(TAG, "mcp2515_read_status");
    esp_err_t ret = spi_device_transmit(dev->spi, &t);
    mcp2515_unlock();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_device_transmit failed in read_status: %d", ret);
        return 0xFF;
    }
    ESP_LOGI(TAG, "mcp2515_read_status: val=0x%02X", rx[1]);
    return rx[1];
}

esp_err_t mcp2515_set_mode(mcp2515_t *dev, uint8_t mode) {
    esp_err_t ret = mcp2515_bit_modify(dev, MCP2515_REG_CANCTRL, 0xE0, mode);
    vTaskDelay(pdMS_TO_TICKS(10));
    uint8_t canstat = mcp2515_read_reg(dev, MCP2515_REG_CANSTAT);
    uint8_t canctrl = mcp2515_read_reg(dev, MCP2515_REG_CANCTRL);
    ESP_LOGI(TAG, "mcp2515_set_mode: CANSTAT=0x%02X (%s) CANCTRL=0x%02X", canstat, mcp2515_mode_str(canstat), canctrl);
    return ret;
}


// Bit timing values for 16MHz MCP2515 (SJW=1, sample at 75%)
typedef struct { uint32_t bitrate; uint8_t cnf1, cnf2, cnf3; } mcp2515_bitrate_config_t;
static const mcp2515_bitrate_config_t bitrate_table[] = {
    {1000000, 0x00, 0xD0, 0x82}, // 1 Mbps
    {500000,  0x01, 0xB1, 0x85}, // 500 kbps
    {250000,  0x03, 0xB1, 0x85}, // 250 kbps
    {125000,  0x07, 0xB1, 0x85}, // 125 kbps
    {100000,  0x09, 0xB1, 0x85}, // 100 kbps
    {50000,   0x13, 0xB1, 0x85}, // 50 kbps
    {20000,   0x31, 0xB1, 0x85}, // 20 kbps
    {10000,   0x63, 0xB1, 0x85}, // 10 kbps
};

esp_err_t mcp2515_config_bitrate(mcp2515_t *dev, uint32_t bitrate, uint8_t clk_mhz) {
    if (clk_mhz != 16) return ESP_ERR_INVALID_ARG;
    mcp2515_set_mode(dev, MCP2515_MODE_CONFIG);
    const mcp2515_bitrate_config_t *cfg = NULL;
    for (size_t i = 0; i < sizeof(bitrate_table)/sizeof(bitrate_table[0]); ++i) {
        if (bitrate_table[i].bitrate == bitrate) { cfg = &bitrate_table[i]; break; }
    }
    if (!cfg) return ESP_ERR_INVALID_ARG;
    mcp2515_write_reg(dev, MCP2515_REG_CNF1, cfg->cnf1);
    mcp2515_write_reg(dev, MCP2515_REG_CNF2, cfg->cnf2);
    mcp2515_write_reg(dev, MCP2515_REG_CNF3, cfg->cnf3);
    mcp2515_set_mode(dev, MCP2515_MODE_NORMAL);
    return ESP_OK;
}

// Set RXB0 mask/filter (standard/extended ID: see datasheet for format)
esp_err_t mcp2515_set_filter_mask(mcp2515_t *dev, const uint8_t mask[4], const uint8_t filter[4]) {
    mcp2515_set_mode(dev, MCP2515_MODE_CONFIG);
    // RXM0SIDH @ 0x20, RXF0SIDH @ 0x00
    for (int i = 0; i < 4; ++i) {
        mcp2515_write_reg(dev, 0x20 + i, mask[i]); // RXM0
        mcp2515_write_reg(dev, 0x00 + i, filter[i]); // RXF0
    }
    mcp2515_set_mode(dev, MCP2515_MODE_NORMAL);
    return ESP_OK;
}

esp_err_t mcp2515_set_loopback(mcp2515_t *dev) {
    return mcp2515_set_mode(dev, MCP2515_MODE_LOOPBACK);
}
