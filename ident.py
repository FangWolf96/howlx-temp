import board
import microcontroller
import time
from adafruit_bme280 import basic as adafruit_bme280

# --------------- Settings ----------------
SENSOR_NAME = "HowlX Atmos"   # your chosen name

# --------------- I2C Setup ---------------
i2c = board.I2C()  # uses board.SCL / board.SDA

# --------------- Unique ID ---------------
# Normalize to hex string (last 3 bytes)
unique_id = ["{:02x}".format(int(x)) for x in microcontroller.cpu.uid]
sensor_id = "".join(unique_id[-3:])

# --------------- Probe for BME280 ---------------
def find_bme(i2c_obj):
    while not i2c_obj.try_lock():
        pass
    try:
        addrs = i2c_obj.scan()
    finally:
        i2c_obj.unlock()

    if 0x76 in addrs:
        addr = 0x76
    elif 0x77 in addrs:
        addr = 0x77
    else:
        return None, addrs, None

    try:
        sensor = adafruit_bme280.Adafruit_BME280_I2C(i2c_obj, address=addr)
        return sensor, addrs, addr
    except Exception:
        return None, addrs, addr

bme280, seen, addr = find_bme(i2c)

# --------------- Print Info --------------
print(f"{SENSOR_NAME} #{sensor_id} ready")

if bme280 is not None:
    print("BME280 found at 0x%02X" % addr)
    print("Temperature: %.1f °C" % bme280.temperature)
    print("Humidity: %.1f %%" % bme280.humidity)
    print("Pressure: %.1f hPa" % bme280.pressure)
else:
    human_addrs = [("0x%02X" % a) for a in (seen or [])]
    print("No BME280 detected on I2C (looked for 0x76/0x77).")
    print("I2C scan saw:", human_addrs if human_addrs else "[]")
    print("Check SDA/SCL wiring, power (3.3V), and pull-ups.")
