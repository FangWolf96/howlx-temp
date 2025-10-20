# HowlX Atmos Sensor (Feather ESP32-S2/S3 + BME280 / BME680 / SHT3x + MAX17048)

**CircuitPython firmware for a low-power environmental telemetry node that:**

- 🧠 Automatically detects connected sensors (**BME280**, **BME680**, or **SHT3x**)
- 📡 Reads temperature / humidity / pressure / gas resistance (if available)
- 🔋 Monitors Li-ion battery voltage and percentage via **MAX17048**
- ☁️ Posts a batch to **Adafruit IO (AIO Group)** and **InfluxDB v2/v1**
- 🌡️ Derives dew point, wet-bulb, humidity ratio, enthalpy, altitude
- 💾 Persists last battery readings across deep sleep for charging-state inference
- 🛡️ Uses watchdog + retry wrappers for high reliability
- ⚙️ Supports per-board calibration via `/bme_offsets.json`  
  (auto-fetches calibration file from repo if missing)
- 🐺 Generates unique identity tags per device — e.g. `[SHT30:S2-FE74A8]`
- 💤 Designed for ultra-low power: reads sensors before Wi-Fi, then deep sleeps

---

# Hardware

***MCU: Adafruit Feather ESP32-S2 / ESP32-S3***  
*(Auto-detected via platform inspection)*

***Env Sensors: BME280 / BME680 / SHT30 (0x76/0x77 or 0x44/0x45)***  
• BME280 – Temperature / Humidity / Pressure  
• BME680 – Temperature / Humidity / Pressure / Gas Resistance  
• SHT30 – Temperature / Humidity (heater off by default)

***Fuel Gauge: MAX17048 / MAX17049 (0x36)***  
• Monitors Li-ion voltage and charge %  
• Infers charging / charged / discharging state

***Power: Single-cell Li-ion / LiPo***  
• Powered via Feather JST or USB  
• Supports solar or external charging

---

# Breakout Wiring (if used)

| **Sensor Pin** | **Feather Pin** | **Notes** |
|-----------------|-----------------|-----------|
| VIN / VCC | 3V3 | 3.3 V supply |
| GND | GND | Common ground |
| SCL | SCL | I²C clock |
| SDA | SDA | I²C data |
| CSB / SDO | Float or tie to select address | BME280/BME680 address select |

---

# Features

- Dynamic board detection (**S2 / S3 / RP2040 / UNK**)
- Firmware version reporting (`fw-version` feed)
- Optional InfluxDB logging (`INFLUX_ENABLE=1`)
- Adafruit IO Group integration (`AIO_ENABLE=1`)
- GitHub offset file auto-fetch on first boot
- 5-minute deep-sleep cycle by default (`SLEEP_SECONDS=300`)
- Self-heating minimized by limited sensor access before Wi-Fi init

---

