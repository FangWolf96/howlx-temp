import time, math, ssl
import board, busio
import wifi, socketpool
import adafruit_requests
from os import getenv
from adafruit_bme280 import basic as adafruit_bme280

# ---------- Settings ----------
WIFI_SSID = getenv("WIFI_SSID")
WIFI_PASSWORD = getenv("WIFI_PASSWORD")
AIO_USER = getenv("ADAFRUIT_AIO_USERNAME")
AIO_KEY  = getenv("ADAFRUIT_AIO_KEY")
AIO_GROUP = getenv("AIO_GROUP_KEY") or "howlx-proto-board-001"

SLEEP_SECONDS = 300         # 5 minutes
SEA_LEVEL_HPA = 1013.25     # set to your local sea-level pressure for better altitude

# ---------- Psychrometric helpers ----------
def dewpoint_c(temp_c, rh):
    a, b = 17.62, 243.12
    gamma = (a * temp_c) / (b + temp_c) + math.log(rh / 100.0)
    return (b * gamma) / (a - gamma)

def wetbulb_c(temp_c, rh):
    # Stull (2011) approximation
    return (
        temp_c * math.atan(0.151977 * (rh + 8.313659) ** 0.5)
        + math.atan(temp_c + rh)
        - math.atan(rh - 1.676331)
        + 0.00391838 * (rh ** 1.5) * math.atan(0.023101 * rh)
        - 4.686035
    )

def humidity_ratio(temp_c, rh, pressure_hpa=1013.25):
    es = 6.112 * math.exp((17.67 * temp_c) / (temp_c + 243.5))  # hPa
    e = (rh / 100.0) * es
    return 0.62198 * e / (pressure_hpa - e)  # kg/kg

def enthalpy(temp_c, w):
    return 1.006 * temp_c + w * (2501 + 1.805 * temp_c)  # kJ/kg dry air

# ---------- Read sensor BEFORE Wi-Fi (reduce self-heating) ----------
i2c = busio.I2C(board.SCL, board.SDA)
sensor = None
while sensor is None:
    while not i2c.try_lock():
        pass
    addrs = [hex(x) for x in i2c.scan()]
    i2c.unlock()
    if "0x76" in addrs:
        sensor = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)
    elif "0x77" in addrs:
        sensor = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x77)
    else:
        print("No BME280 found; I2C:", addrs, "retrying in 2s…")
        time.sleep(2)

sensor.sea_level_pressure = SEA_LEVEL_HPA

temp_c = float(sensor.temperature)
rh = float(sensor.relative_humidity)
pressure_hpa = float(sensor.pressure)
alt_m = float(sensor.altitude)

# Derived metrics
temp_f = temp_c * 9 / 5 + 32
dp_c = dewpoint_c(temp_c, rh)
dp_f = dp_c * 9 / 5 + 32
wb_c = wetbulb_c(temp_c, rh)
wb_f = wb_c * 9 / 5 + 32
w = humidity_ratio(temp_c, rh, pressure_hpa)
h = enthalpy(temp_c, w)

print("==== HowlX Readings ====")
print(f"Dry Bulb: {temp_c:.2f} °C / {temp_f:.2f} °F")
print(f"Dew Point: {dp_c:.2f} °C / {dp_f:.2f} °F")
print(f"Wet Bulb: {wb_c:.2f} °C / {wb_f:.2f} °F")
print(f"Humidity: {rh:.2f} %")
print(f"Pressure: {pressure_hpa:.2f} hPa")
print(f"Altitude: {alt_m:.2f} m")
print(f"Humidity Ratio: {w:.5f} kg/kg")
print(f"Enthalpy: {h:.2f} kJ/kg")
print("========================")

# ---------- Wi-Fi + HTTP session ----------
print("Connecting Wi-Fi…")
wifi.radio.connect(WIFI_SSID, WIFI_PASSWORD)
pool = socketpool.SocketPool(wifi.radio)
requests = adafruit_requests.Session(pool, ssl.create_default_context())
print("Wi-Fi OK:", wifi.radio.ipv4_address)

# ---------- Adafruit IO Group batch POST ----------
# Endpoint: POST /api/v2/{username}/groups/{group_key}/data
# Payload form: {"feeds":[{"key":"<feed-key>","value":...}, ...]}
BASE = f"https://io.adafruit.com/api/v2/{AIO_USER}"
URL  = f"{BASE}/groups/{AIO_GROUP}/data"
HEADERS = {"X-AIO-Key": AIO_KEY, "Content-Type": "application/json"}

feeds_payload = {
    "feeds": [
        {"key": "temperature-c",         "value": round(temp_c, 2)},
        {"key": "temperature-f",         "value": round(temp_f, 2)},
        {"key": "dewpoint-c",            "value": round(dp_c, 2)},
        {"key": "dewpoint-f",            "value": round(dp_f, 2)},
        {"key": "wetbulb-c",             "value": round(wb_c, 2)},
        {"key": "wetbulb-f",             "value": round(wb_f, 2)},
        {"key": "humidity-pct",          "value": round(rh, 2)},
        {"key": "pressure-hpa",          "value": round(pressure_hpa, 2)},
        {"key": "altitude-m",            "value": round(alt_m, 2)},
        {"key": "humidity-ratio-kgkg",   "value": round(w, 5)},
        {"key": "enthalpy-kjkg",         "value": round(h, 2)},
    ]
}

try:
    r = requests.post(URL, json=feeds_payload, headers=HEADERS, timeout=15)
    print("AIO group POST status:", r.status_code)
    # If 404, ensure the group and each feed key exist (exact spelling, hyphens only)
    r.close()
except Exception as e:
    print("AIO POST failed:", e)

# ---------- Deep sleep (best battery life) ----------
try:
    import alarm
    print(f"Sleeping for {SLEEP_SECONDS} s…")
    t = alarm.time.TimeAlarm(monotonic_time=time.monotonic() + SLEEP_SECONDS)
    alarm.exit_and_deep_sleep_until_alarms(t)
except Exception as e:
    print("Deep sleep not available, fallback loop:", e)
    time.sleep(SLEEP_SECONDS)
