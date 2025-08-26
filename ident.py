import board, busio
import microcontroller
from adafruit_bme280 import basic as adafruit_bme280

# --------------- Settings ----------------
SENSOR_NAME = "HowlX Atmos"   # your chosen name

# --------------- I2C Setup ---------------
i2c = board.I2C()
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

# --------------- Unique ID ---------------
# Always normalize to integer before hex formatting
unique_id = ["{:02x}".format(int(x)) for x in microcontroller.cpu.uid]
sensor_id = "".join(unique_id[-3:])  # shorten to last 3 bytes

# --------------- Print Info --------------
print(f"{SENSOR_NAME} #{sensor_id} ready")
print("Temperature: %.1f °C" % bme280.temperature)
print("Humidity: %.1f %%" % bme280.humidity)
print("Pressure: %.1f hPa" % bme280.pressure)
