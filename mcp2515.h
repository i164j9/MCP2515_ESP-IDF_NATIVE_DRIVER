#ifndef MCP2515_H
#define MCP2515_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	spi_device_handle_t spi;
	int cs_gpio;
	// Add more fields as needed
} mcp2515_t;

// --- CAN_frame_t and related definitions ---
#define CAN_FRAME_MAX_LEN 8

typedef enum {
    CAN_frame_std = 0,
    CAN_frame_ext = 1
} CAN_frame_format_t;

typedef enum {
    CAN_no_RTR = 0,
    CAN_RTR = 1
} CAN_rtr_t;

typedef struct {
    uint32_t MsgID;
    CAN_frame_format_t FMT;
    CAN_rtr_t RTR;
    uint8_t DLC;
    uint8_t data[CAN_FRAME_MAX_LEN];
} CAN_frame_t;



typedef enum {
    Single_Mode = 0,
    Dual_Mode = 1
} CAN_filter_mode_t;

typedef struct {
    CAN_filter_mode_t FM;
    uint8_t ACR0;
    uint8_t ACR1;
    uint8_t ACR2;
    uint8_t ACR3;
    uint8_t AMR0;
    uint8_t AMR1;
    uint8_t AMR2;
    uint8_t AMR3;
} CAN_filter_t;

// Core driver API
esp_err_t mcp2515_init(mcp2515_t *dev, spi_host_device_t host, int cs_gpio);
esp_err_t mcp2515_reset(mcp2515_t *dev);
uint8_t   mcp2515_read_reg(mcp2515_t *dev, uint8_t reg);
esp_err_t mcp2515_write_reg(mcp2515_t *dev, uint8_t reg, uint8_t val);
esp_err_t mcp2515_bit_modify(mcp2515_t *dev, uint8_t reg, uint8_t mask, uint8_t data);
uint8_t   mcp2515_read_status(mcp2515_t *dev);
esp_err_t mcp2515_set_mode(mcp2515_t *dev, uint8_t mode);

// Bitrate config for common CAN speeds (clk_mhz = 16 for MCP2515)
esp_err_t mcp2515_config_bitrate(mcp2515_t *dev, uint32_t bitrate, uint8_t clk_mhz);

// Set RX filter/mask for RXB0 (mask/filter are 4 bytes each)
esp_err_t mcp2515_set_filter_mask(mcp2515_t *dev, const uint8_t mask[4], const uint8_t filter[4]);

// Set MCP2515 to loopback mode
esp_err_t mcp2515_set_loopback(mcp2515_t *dev);

// High-level CAN frame TX/RX API for bridge
int mcp2515_write_frame(mcp2515_t *dev, const CAN_frame_t *frame);
int mcp2515_read_frame(mcp2515_t *dev, CAN_frame_t *frame);

// Pin definitions for MCP2515 SPI interface and INT pin
#define MCP2515_SCK_PIN   18
#define MCP2515_MISO_PIN  19
#define MCP2515_MOSI_PIN  23
#define MCP2515_CS_PIN    5
#define MCP2515_INT_PIN   15

// MCP2515 SPI commands
#define MCP2515_CMD_RESET         0xC0
#define MCP2515_CMD_READ          0x03
#define MCP2515_CMD_READ_RX_BUF_0 0x90
#define MCP2515_CMD_READ_RX_BUF_1 0x94
#define MCP2515_CMD_WRITE         0x02
#define MCP2515_CMD_LOAD_TX_BUF_0 0x40
#define MCP2515_CMD_LOAD_TX_BUF_1 0x42
#define MCP2515_CMD_LOAD_TX_BUF_2 0x44
#define MCP2515_CMD_RTS_TXB0      0x81
#define MCP2515_CMD_RTS_TXB1      0x82
#define MCP2515_CMD_RTS_TXB2      0x84
#define MCP2515_CMD_READ_STATUS   0xA0
#define MCP2515_CMD_RX_STATUS     0xB0
#define MCP2515_CMD_BIT_MODIFY    0x05

// MCP2515 register addresses (partial)
#define MCP2515_REG_CANSTAT   0x0E
#define MCP2515_REG_CANCTRL   0x0F
#define MCP2515_REG_CNF1      0x2A
#define MCP2515_REG_CNF2      0x29
#define MCP2515_REG_CNF3      0x28
#define MCP2515_REG_CANINTE   0x2B
#define MCP2515_REG_CANINTF   0x2C
#define MCP2515_REG_EFLG      0x2D
#define MCP2515_REG_TXB0CTRL  0x30
#define MCP2515_REG_TXB1CTRL  0x40
#define MCP2515_REG_TXB2CTRL  0x50
#define MCP2515_REG_RXB0CTRL  0x60
#define MCP2515_REG_RXB1CTRL  0x70

// CANCTRL mode bits
#define MCP2515_MODE_NORMAL   0x00
#define MCP2515_MODE_SLEEP    0x20
#define MCP2515_MODE_LOOPBACK 0x40
#define MCP2515_MODE_LISTEN   0x60
#define MCP2515_MODE_CONFIG   0x80

#ifdef __cplusplus
}
#endif

#endif // MCP2515_H
