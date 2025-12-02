# Sovelar
Sovelar is a MPPT non-synchronous buck converter with Esp32, WiFi capability and Interactive 20x4 LCD Display

## Feature
-  45V 10A Maximum Power
-  Over/Under Voltage, Over Current, Overheat Protection
-  Varible PWM Fan Control
-  Powerful ESP32
-  Backflow Diode to prevent current flow back to the SolarCell
-  20x4 LCD display with a live data
-  ADS1115 ADC for Precision measurement
-  LM2976 (12V/5V) 3A Regulator for Internal power Rail

## Library requiment 
- `<Wire.h>`
- `<WiFi.h>`
- `<Arduino.h>`
- `<Adafruit_ADS1X15.h>`
- `<LiquidCrystal_I2C.h>`

## Hardware Spec
- MOSFET : [CSD19505](https://www.ti.com/lit/ds/symlink/csd19505kcs.pdf)
- MOSFET Driver : [MCP1416](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/20002092G.pdf)
- Flyback & Backflow Diode : [MBR20100CT](https://www.onsemi.com/pdf/datasheet/mbr20100ct-d.pdf)
- Internal Power Rail : [LM2576](https://www.ti.com/lit/ds/symlink/lm2576hv.pdf)
- MCU : [ESP32 wroom-32](https://documentation.espressif.com/esp32-wroom-32_datasheet_en.pdf)
- Bulk Capacitor (IN&OUT) : 220uF 100v 
