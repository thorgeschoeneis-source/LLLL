# Flashing Guide

This project targets an ESP32-C6 with a Quervo DWM3000 module.

## Pin mapping

- IRQ: GPIO5
- RESET: GPIO14
- WAKEUP: GPIO15
- MOSI: GPIO7
- MISO: GPIO4
- CLK: GPIO3
- CS: GPIO6

## Flashing

```powershell
cmd /c "call C:\esp\v6.0\esp-idf\export.bat && cd /d C:\Users\mikao\Documents\GitHub\LLLL && idf.py flash monitor"
```

If you only want to rebuild first:

```powershell
cmd /c "call C:\esp\v6.0\esp-idf\export.bat && cd /d C:\Users\mikao\Documents\GitHub\LLLL && idf.py build"
```

## Notes

- Use the ESP-IDF v6.0 environment.
- The project is currently a UWB bringup baseline, not a completed localization stack.
- If the serial output still shows old pins, rebuild and flash again so the generated `sdkconfig` is picked up.
