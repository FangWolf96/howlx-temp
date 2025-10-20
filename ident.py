#Identification Script 1.2 10/20/25 - Add support for BME680
import board
import microcontroller
import time
from adafruit_bme280 import basic as adafruit_bme280
import adafruit_bme680
import adafruit_sht31d

# ---------------- Settings ----------------
SENSOR_NAME = "HowlX Atmos"

# ---------------- I2C Setup ----------------
i2c = board.I2C()  # uses STEMMA QT SDA/SCL

# ---------------- Unique ID ----------------
uid_bytes = microcontroller.cpu.uid
uid_hex = ''.join(['{:02X}'.format(b) for b in uid_bytes])
device_uid = uid_hex[-6:]  # short tag like 8A4C1F

# ---------------- Board Identification ----------------
# Try to extract something descriptive about the board
import sys, os, microcontroller

def detect_board_code():
    # 1) Most reliable across builds
    p = getattr(sys, "platform", "").lower()
    if "esp32s2" in p: return "S2"
    if "esp32s3" in p: return "S3"
    if "rp2040" in p or p == "rp2": return "RP"

    # 2) Fallback: CPU name (sometimes missing/odd)
    name = getattr(microcontroller.cpu, "name", None)
    if name:
        n = str(name).replace(" ", "").replace("_", "").upper()
        if "ESP32S3" in n or "S3" in n: return "S3"
        if "ESP32S2" in n or "S2" in n: return "S2"
        if "RP2040" in n or "RP2" in n: return "RP"
        if n: return n[:3]

    # 3) Last resort: OS machine string
    try:
        mach = os.uname().machine.lower()
        if "esp32s2" in mach: return "S2"
        if "esp32s3" in mach: return "S3"
        if "rp2040" in mach: return "RP"
    except Exception:
        pass

    return "UNK"

board_code = detect_board_code()

# ---------------- I2C Scan ----------------
def scan_i2c(i2c_obj):
    while not i2c_obj.try_lock():
        pass
    try:
        addrs = i2c_obj.scan()
    finally:
        i2c_obj.unlock()
    return addrs

seen = scan_i2c(i2c)

# ---------------- Probe Functions ----------------
def try_bme280(i2c_obj, addrs):
    for addr in (0x76, 0x77):
        if addr in addrs:
            try:
                return adafruit_bme280.Adafruit_BME280_I2C(i2c_obj, address=addr), addr
            except Exception:
                pass
    return None, None

def try_bme680(i2c_obj, addrs):
    for addr in (0x76, 0x77):
        if addr in addrs:
            try:
                return adafruit_bme680.Adafruit_BME680_I2C(i2c_obj, address=addr), addr
            except Exception:
                pass
    return None, None

def try_sht3x(i2c_obj, addrs):
    for addr in (0x44, 0x45):
        if addr in addrs:
            try:
                return adafruit_sht31d.SHT31D(i2c_obj, address=addr), addr
            except Exception:
                pass
    return None, None

# ---------------- Detect Sensors ----------------
bme280, bme280_addr = try_bme280(i2c, seen)
bme680, bme680_addr = try_bme680(i2c, seen)
sht3x, sht_addr = try_sht3x(i2c, seen)

# ---------------- Determine Identity ----------------
if bme680:
    sensor_type = "BME680"
    addr = bme680_addr
elif bme280:
    sensor_type = "BME280"
    addr = bme280_addr
elif sht3x:
    sensor_type = "SHT30"
    addr = sht_addr
else:
    sensor_type = "UNKNOWN"
    addr = None

identity = f"{SENSOR_NAME} [{sensor_type}:{board_code}-{device_uid}]"

# ---------------- Display Info ----------------
print(identity)
human_addrs = [("0x%02X" % a) for a in (seen or [])]
print("I2C scan saw:", human_addrs if human_addrs else "[]")
print("HowlX Atmos Identification Script v1.2 — 2025-10-20")

if bme680:
    print(f"BME680 found at 0x{addr:02X}")
    print(f"T: {bme680.temperature:.2f} °C, H: {bme680.humidity:.2f} %, P: {bme680.pressure:.1f} hPa, Gas: {bme680.gas:.0f} Ω")
elif bme280:
    print(f"BME280 found at 0x{addr:02X}")
    print(f"T: {bme280.temperature:.2f} °C, H: {bme280.humidity:.2f} %, P: {bme280.pressure:.1f} hPa")
elif sht3x:
    print(f"SHT3x found at 0x{addr:02X}")
    print(f"T: {sht3x.temperature:.2f} °C, H: {sht3x.relative_humidity:.2f} %")
else:
    print("No supported sensor detected (BME280/BME680 0x76/0x77 or SHT30 0x44/0x45).")
    print("Check wiring and power.")


# --------------- Enable Read Loop -
# If you want continuous logging, pick whichever was found first:
# sensor_temp = None
# sensor_hum = None
# if bme280: sensor_temp = lambda: bme280.temperature; sensor_hum = lambda: bme280.humidity
# elif sht3x: sensor_temp = lambda: sht3x.temperature; sensor_hum = lambda: sht3x.relative_humidity
# while sensor_temp and sensor_hum:
#     t = sensor_temp(); h = sensor_hum()
#     print(f"{time.monotonic():.1f},ID={sensor_id},T={t:.2f}C,H={h:.2f}%")
#     time.sleep(5)
