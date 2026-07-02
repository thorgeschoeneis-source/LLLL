# ESP32-C6 UWB Tag Bringup

This repository contains a minimal ESP-IDF 6.0 project for an ESP32-C6 with a Quervo DWM3000 module. The current focus is a clean hardware bringup and a stable UWB tag baseline.

## Current wiring

| Signal | ESP32-C6 GPIO |
| --- | --- |
| DW3000 IRQ | GPIO5 |
| DW3000 RESET | GPIO14 |
| DW3000 WAKEUP | GPIO15 |
| DW3000 MOSI | GPIO7 |
| DW3000 MISO | GPIO4 |
| DW3000 CLK | GPIO3 |
| DW3000 CS | GPIO6 |

## Build and flash

```bash
idf.py build
idf.py -p COM11 flash monitor
```

If your ESP-IDF shell is not already active on Windows, use:

```powershell
cmd /c "call C:\esp\v6.0\esp-idf\export.bat && cd /d C:\Users\mikao\Documents\GitHub\LLLL && idf.py build"
```

## Structure

- `main/main.c` - application entrypoint and UWB bringup
- `sdkconfig.defaults` - default configuration for the ESP32-C6 + DWM3000 setup
- `components/decadriver` - DW3000 platform layer
- `components/libdeca` - libdeca integration and ranging helpers

## Status

The project currently boots the DW3000 stack and is being rebuilt toward a clean tag-oriented UWB workflow.
