# UWB3000 Communication Example

This project demonstrates communication between two ESP32-C6 boards using UWB3000 (DW3000) modules.

## Hardware Setup

Each board needs:
- ESP32-C6 microcontroller
- UWB3000 module
- Connections as per pin mapping below

## Pin Mapping

| ESP32-C6 Pin | UWB3000 Pin | Function |
|-------------|-------------|----------|
| GPIO3      | SCLK        | SPI Clock |
| GPIO4      | MISO        | SPI Master In Slave Out |
| GPIO5      | IRQ         | Interrupt Request |
| GPIO6      | CS          | SPI Chip Select |
| GPIO7      | MOSI        | SPI Master Out Slave In |
| GPIO14     | RESET       | Reset signal |
| GPIO15     | WAKEUP      | Wakeup signal |
| GPIO8      | -           | Status LED |

## Board Configuration

### Board 1 (Master)
In `main/main.c`, set:
```c
#define BOARD_IS_MASTER    true
```

### Board 2 (Slave)
In `main/main.c`, set:
```c
#define BOARD_IS_MASTER    false
```

## How It Works

1. **Master Board**: Sends "PING" messages every 2 seconds
2. **Slave Board**: Listens for messages and responds with "PONG"
3. Both boards blink their LEDs when communication occurs
4. Serial output shows communication status

## Building and Flashing

1. Configure ESP-IDF environment
2. Build: `idf.py build`
3. Flash: `idf.py flash`
4. Monitor: `idf.py monitor`

## Expected Output

### Master Board:
```
I (1234) UWB_COMM: Board configured as MASTER (address 0x0100)
I (2234) UWB_COMM: Sending PING (seq: 0)
I (3234) UWB_COMM: Received response: PONG (seq: 0)
I (4234) UWB_COMM: Sending PING (seq: 1)
...
```

### Slave Board:
```
I (1234) UWB_COMM: Board configured as SLAVE (address 0x0200)
I (2234) UWB_COMM: Received: PING (seq: 0)
I (2234) UWB_COMM: Sending PONG (seq: 0)
I (4234) UWB_COMM: Received: PING (seq: 1)
...
```

## Troubleshooting

- Ensure antennas are properly connected
- Check power supply (3.3V)
- Verify SPI connections
- Make sure boards are within UWB range (up to ~100m line-of-sight)
- Check serial output for error messages

## Files

- `main/main.c` - Main application with UWB communication
- `main/dw3000.h` - DW3000 register definitions and function declarations
- `main/dw3000.c` - DW3000 driver implementation
- `main/CMakeLists.txt` - Build configuration

