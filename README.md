# HowlX Atmos Sensor (Feather ESP32-S2 + BME280 + MAX17048)

**CircuitPython firmware for a low-power environmental node that:**

 - Reads BME280 temperature / humidity / pressure

 - Reads MAX17048 Li-ion fuel gauge (voltage & %)

 - Posts a batch to Adafruit IO (AIO Group)

 - Derives dew point, wet-bulb, humidity ratio, enthalpy, altitude

 - Persists last battery values across deep sleep for charging state inference

 - Has watchdog + retry wrappers for robustness

 - Supports per-board calibration via /bme_offsets.json

 - Can auto-fetch calibration from this repo if the file is missing

 - Designed to minimize self-heating: the BME280 is read before Wi-Fi and deep sleeps between updates.

# Hardware

***MCU: Adafruit Feather ESP32-S2***

***Env Sensor: BME280 (onboard or breakout at 0x76/0x77)***

***Fuel Gauge: MAX17048/49 (0x36)***

***Power: Single-cell Li-ion/LiPo***

**Breakout wiring (if used):**

**BME280**	**Feather**
VIN	      3V3
GND	      GND
SCL	      SCL
SDA	      SDA
CSB/SDO	(float or tie for address select)