# MCP2515 ESP-IDF Driver Library

This library provides a native ESP-IDF driver for the MCP2515 CAN controller, supporting robust CAN communication for ESP32 and compatible platforms.

## Features
- Full MCP2515 SPI command set (RESET, READ, WRITE, BIT MODIFY, LOAD/RTS, etc.)
- CAN frame TX/RX (standard and extended)
- Bitrate configuration for common CAN speeds
- RX filter and mask configuration
- Loopback, normal, listen, and sleep modes
- Thread-safe, FreeRTOS-friendly
- Example integration with Bluetooth SPP bridge

## Getting Started

### 1. Add Source Files
Copy the following files into your ESP-IDF project's `main/` directory:
- `mcp2515.c`
- `include/mcp2515.h`

Include them in your `CMakeLists.txt` or `component.mk` as needed.

### 2. Hardware Connections
- Connect MCP2515 SPI pins (MOSI, MISO, SCK, CS) to ESP32 (default: MOSI=23, MISO=19, SCK=18, CS=5)
- Connect INT pin to a GPIO (default: 15)
- Ensure MCP2515 is powered at 5V (with logic level shifting if needed)

### 3. Initialization Example
```c
#include "mcp2515.h"

mcp2515_t mcp2515_dev;
// Initialize SPI and MCP2515
ESP_ERROR_CHECK(mcp2515_init(&mcp2515_dev, VSPI_HOST, MCP2515_CS_PIN));
// Set CAN bitrate (e.g., 500k, 16MHz clock)
ESP_ERROR_CHECK(mcp2515_config_bitrate(&mcp2515_dev, 500000, 16));
// (Optional) Set loopback mode for testing
// ESP_ERROR_CHECK(mcp2515_set_loopback(&mcp2515_dev));
```

### 4. Sending a CAN Frame
```c
CAN_frame_t frame = {
    .MsgID = 0x123,
    .FMT = CAN_frame_std,
    .RTR = CAN_no_RTR,
    .DLC = 8,
    .data = {0x11,0x22,0x33,0x44,0x55,0x66,0x77,0x88}
};
mcp2515_write_frame(&mcp2515_dev, &frame);
```

### 5. Receiving a CAN Frame
```c
CAN_frame_t rx_frame;
if (mcp2515_read_frame(&mcp2515_dev, &rx_frame) == 0) {
    // Process rx_frame
}
```

### 6. Setting RX Filter/Mask
```c
// Accept only 0x123 (standard ID)
uint8_t mask[4] = {0xFF, 0xE0, 0, 0};
uint8_t filter[4] = {0x24, 0x60, 0, 0};
mcp2515_set_filter_mask(&mcp2515_dev, mask, filter);
```

### 7. Interrupt Handling
- Configure the INT pin as input with pull-up and attach an ISR handler.
- Use the provided ISR example in `main.c` for minimal latency.

### 8. Supported Bitrates
- 1M, 500k, 250k, 125k, 100k, 50k, 20k, 10k (for 16MHz MCP2515)

## API Reference
See `mcp2515.h` for all available functions and configuration options.

## Example Project
See the provided `main.c` for a full example including Bluetooth SPP bridging and CAN task management.

## License
MIT or Apache 2.0 (choose your preferred license)

---
For questions or contributions, open an issue or pull request.
