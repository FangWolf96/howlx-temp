# code.py — HowlX proto board (Feather ESP32-S2 + BME280 + MAX17048)
# v1.0
#  - Reads BME280 (0x76/0x77)
# - Reads MAX17048 fuel gauge (0x36)
# - Posts to Adafruit IO Group (batch)
# - Infers charging/discharging/charged and persists last %/V across deep sleep
# v1.1
# - Per-board offsets (loaded from /bme_offsets.json)
# - Calibration mode PRINTS offsets JSON (no file writes)
# - Restores full AIO payload (°C/°F, dewpoint, wetbulb, enthalpy, humidity ratio, altitude, battery, charging-state)
# - Publishes offset values to feed and sets offset binary value 1 = Calibrated 0 = Not Calibrated
# 
import time, math, ssl, struct, supervisor, json
import board, busio
import wifi, socketpool
import adafruit_requests
from os import getenv
from adafruit_bme280 import basic as adafruit_bme280
import adafruit_max1704x

# -------- Settings --------
WIFI_SSID = getenv("WIFI_SSID")
WIFI_PASSWORD = getenv("WIFI_PASSWORD")
AIO_USER = getenv("ADAFRUIT_AIO_USERNAME")
AIO_KEY  = getenv("ADAFRUIT_AIO_KEY")
AIO_GROUP = getenv("AIO_GROUP_KEY")              # e.g. howlx-proto-board-001

SLEEP_SECONDS = 300                              # 5 minutes
SEA_LEVEL_HPA = 1013.25                          # for better altitude calc

# -------- Offsets / Calibration --------
CAL_FILE = "/bme_offsets.json"                   # you maintain this file manually
DO_CALIBRATE = False                             # True => prints offsets JSON then exits
CAL_SECONDS  = 60

# Pull reference from your other board’s AIO group
REF_FROM_AIO = True
AIO_REF_GROUP = getenv("AIO_REF_GROUP") or "howlx-proto-board-002"

# If REF_FROM_AIO=False, fill these:
REF_TEMP_C = None
REF_RH_PCT = None
REF_P_HPA  = None

# -------- Psychrometric helpers --------
def dewpoint_c(t, rh):
    a, b = 17.62, 243.12
    rh = max(0.01, min(100.0, rh))
    g = (a * t) / (b + t) + math.log(rh/100.0)
    return (b * g) / (a - g)

def wetbulb_c(t, rh):
    return (t * math.atan(0.151977 * (rh + 8.313659) ** 0.5)
            + math.atan(t + rh)
            - math.atan(rh - 1.676331)
            + 0.00391838 * (rh ** 1.5) * math.atan(0.023101 * rh)
            - 4.686035)

def humidity_ratio(t, rh, p_hpa):
    es = 6.112 * math.exp(17.67 * t / (t + 243.5))
    e  = min((rh/100.0) * es, p_hpa - 0.1)
    return 0.62198 * e / (p_hpa - e)

def enthalpy(t, w):  # kJ/kg dry air
    return 1.006 * t + w * (2501 + 1.805 * t)

def c_to_f(c): return c * 9/5 + 32

# -------- Define offsets calibration values --------

def calibrated_flag(off):
    return 1 if (
        abs(off.get("temp", 0.0))  > 0.01 or
        abs(off.get("hum", 0.0))   > 0.01 or
        abs(off.get("press", 0.0)) > 0.01
    ) else 0

# -------- Load offsets file (manual maintenance) --------
def load_offsets():
    try:
        with open(CAL_FILE, "r") as f:
            d = json.load(f)
            return {"temp": float(d.get("temp", 0.0)),
                    "hum":  float(d.get("hum", 0.0)),
                    "press":float(d.get("press", 0.0))}
    except Exception:
        return {"temp": 0.0, "hum": 0.0, "press": 0.0}

# -------- I2C + BME --------
i2c = board.I2C()
sensor = None
while sensor is None:
    while not i2c.try_lock(): pass
    addrs = [hex(x) for x in i2c.scan()]
    i2c.unlock()
    if "0x76" in addrs:
        sensor = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)
    elif "0x77" in addrs:
        sensor = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x77)
    else:
        print("No BME280 found; retrying in 2s…")
        time.sleep(2)

sensor.sea_level_pressure = SEA_LEVEL_HPA
try:
    sensor.iir_filter = adafruit_bme280.IIR_FILTER_X4
    sensor.standby_period = adafruit_bme280.STANDBY_TC_1000
    sensor.overscan_temperature = adafruit_bme280.OVERSCAN_X1
    sensor.overscan_humidity    = adafruit_bme280.OVERSCAN_X1
    sensor.overscan_pressure    = adafruit_bme280.OVERSCAN_X1
except Exception:
    pass

# -------- Wi-Fi / HTTP --------
print("Connecting Wi-Fi…")
wifi.radio.connect(WIFI_SSID, WIFI_PASSWORD)
pool = socketpool.SocketPool(wifi.radio)
requests = adafruit_requests.Session(pool, ssl.create_default_context())
print("Wi-Fi OK:", wifi.radio.ipv4_address)

def get_ref_from_aio():
    base = f"https://io.adafruit.com/api/v2/{AIO_USER}"
    url  = f"{base}/groups/{AIO_REF_GROUP}"
    headers = {"X-AIO-Key": AIO_KEY}
    r = requests.get(url, headers=headers, timeout=15)
    data = r.json()
    r.close()
    feeds = data.get("feeds", []) or []
    def last_by_suffix(suf):
        for f in feeds:
            k = (f.get("key") or "").lower()
            if k.endswith(suf):
                try: return float(f.get("last_value"))
                except: return None
        return None
    return (last_by_suffix("temperature-c"),
            last_by_suffix("humidity-pct"),
            last_by_suffix("pressure-hpa"))

# -------- Calibration (print-only) --------
offsets = load_offsets()
print("Loaded offsets:", offsets)

if DO_CALIBRATE:
    print("=== CALIBRATION MODE ===")
    Ts, Hs, Ps = [], [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < CAL_SECONDS:
        Ts.append(float(sensor.temperature))
        Hs.append(float(sensor.relative_humidity))
        Ps.append(float(sensor.pressure))
        time.sleep(1.0)
    mean_T = sum(Ts)/len(Ts)
    mean_H = sum(Hs)/len(Hs)
    mean_P = sum(Ps)/len(Ps)

    if REF_FROM_AIO:
        refT, refH, refP = get_ref_from_aio()
        print("Reference from AIO:", refT, refH, refP)
        if None in (refT, refH, refP):
            raise RuntimeError("Ref from AIO missing (temp/hum/press). Check AIO_REF_GROUP & feed keys.")
    else:
        refT, refH, refP = REF_TEMP_C, REF_RH_PCT, REF_P_HPA
        if None in (refT, refH, refP):
            raise RuntimeError("Set REF_* for manual calibration.")

    new_offsets = {
        "temp": round(refT - mean_T, 2),
        "hum":  round(refH - mean_H, 2),
        "press":round(refP - mean_P, 2),
    }
    print(">>> Copy this JSON into /bme_offsets.json on CIRCUITPY:")
    print(json.dumps(new_offsets))
    raise SystemExit  # stop after printing so we don't post partial data

# -------- Apply offsets + compute metrics --------
t_raw = float(sensor.temperature)
rh_raw = float(sensor.relative_humidity)
p_raw  = float(sensor.pressure)

t_c = t_raw + offsets["temp"]
rh  = rh_raw + offsets["hum"]
p_h = p_raw  + offsets["press"]
alt_m = float(sensor.altitude)

dp_c = dewpoint_c(t_c, rh)
wb_c = wetbulb_c(t_c, rh)
w    = humidity_ratio(t_c, rh, p_h)
h_kj = enthalpy(t_c, w)

# -------- Battery + charging state --------
fg = adafruit_max1704x.MAX17048(i2c)
time.sleep(0.1)
v1, p1 = fg.cell_voltage, fg.cell_percent
time.sleep(0.1)
v2, p2 = fg.cell_voltage, fg.cell_percent
vbat = (v1 + v2) / 2.0
bpct = (p1 + p2) / 2.0

def usb_present():
    try:
        return bool(supervisor.runtime.usb_connected)
    except Exception:
        return None

prev_pct = prev_v = None
try:
    import alarm
    mem = alarm.sleep_memory
    if len(mem) >= 8:
        prev_pct, prev_v = struct.unpack("ff", bytes(mem[:8]))
except Exception:
    pass

def infer_state(usb, v_now, p_now, p_prev=None, v_prev=None):
    if usb is True:
        if v_now >= 4.18 or p_now >= 99.0: return "charged"
        if (p_prev is not None and (p_now - p_prev) > 0.05) or \
           (v_prev is not None and (v_now - v_prev) > 0.01): return "charging"
        return "usb-present"
    if usb is False: return "discharging"
    if (p_prev is not None and (p_now - p_prev) > 0.05) or \
       (v_prev is not None and (v_now - v_prev) > 0.01): return "charging"
    return "discharging"

state = infer_state(usb_present(), vbat, bpct, prev_pct, prev_v)

# -------- Post to AIO (full payload) --------
BASE = f"https://io.adafruit.com/api/v2/{AIO_USER}"
URL  = f"{BASE}/groups/{AIO_GROUP}/data"
HEADERS = {"X-AIO-Key": AIO_KEY, "Content-Type": "application/json"}

print("====== HowlX Readings =========================")
print(f"temperature-c      : {round(t_c, 2)}")
print(f"temperature-f      : {round(c_to_f(t_c), 2)}")
print(f"dewpoint-c         : {round(dp_c, 2)}")
print(f"dewpoint-f         : {round(c_to_f(dp_c), 2)}")
print(f"wetbulb-c          : {round(wb_c, 2)}")
print(f"wetbulb-f          : {round(c_to_f(wb_c), 2)}")
print(f"humidity-pct       : {round(rh, 2)}")
print(f"pressure-hpa       : {round(p_h, 2)}")
print(f"altitude-m         : {round(alt_m, 2)}")
print(f"humidity-ratio-kgkg: {round(w, 5)}")
print(f"enthalpy-kjkg      : {round(h_kj, 2)}")
print(f"battery-v          : {round(vbat, 3)}")
print(f"battery-pct        : {round(bpct, 1)}")
print(f"charging-state     : {state}")
print(f"offset-temp        : {round(offsets.get('temp', 0.0), 2)}")
print(f"offset-hum         : {round(offsets.get('hum', 0.0), 2)}")
print(f"offset-press       : {round(offsets.get('press', 0.0), 2)}")
print(f"calibrated         : {calibrated_flag(offsets)}")
print("=============================================")

payload = {
    "feeds": [
        {"key": "temperature-c",         "value": round(t_c, 2)},
        {"key": "temperature-f",         "value": round(c_to_f(t_c), 2)},
        {"key": "dewpoint-c",            "value": round(dp_c, 2)},
        {"key": "dewpoint-f",            "value": round(c_to_f(dp_c), 2)},
        {"key": "wetbulb-c",             "value": round(wb_c, 2)},
        {"key": "wetbulb-f",             "value": round(c_to_f(wb_c), 2)},
        {"key": "humidity-pct",          "value": round(rh, 2)},
        {"key": "pressure-hpa",          "value": round(p_h, 2)},
        {"key": "altitude-m",            "value": round(alt_m, 2)},
        {"key": "humidity-ratio-kgkg",   "value": round(w, 5)},
        {"key": "enthalpy-kjkg",         "value": round(h_kj, 2)},
        {"key": "battery-v",             "value": round(vbat, 3)},
        {"key": "battery-pct",           "value": round(bpct, 1)},
        {"key": "charging-state",        "value": state},

        # --- NEW: publish calibration info ---
        {"key": "offset-temp",           "value": round(offsets.get("temp", 0.0), 2)},
        {"key": "offset-hum",            "value": round(offsets.get("hum", 0.0), 2)},
        {"key": "offset-press",          "value": round(offsets.get("press", 0.0), 2)},
        {"key": "calibrated",            "value": calibrated_flag(offsets)},  # 1 if any offset != 0
    ]
}

try:
    r = requests.post(URL, json=payload, headers=HEADERS, timeout=15)
    print("AIO group POST status:", r.status_code)
    r.close()
except Exception as e:
    print("AIO POST failed:", e)

# -------- Persist last battery values for trend --------
try:
    import alarm
    alarm.sleep_memory[:8] = struct.pack("ff", float(bpct), float(vbat))
except Exception:
    pass

# -------- Deep sleep --------
try:
    import alarm
    print(f"Sleeping for {SLEEP_SECONDS} s…")
    t = alarm.time.TimeAlarm(monotonic_time=time.monotonic() + SLEEP_SECONDS)
    alarm.exit_and_deep_sleep_until_alarms(t)
except Exception as e:
    print("Deep sleep not available, fallback loop:", e)
    time.sleep(SLEEP_SECONDS)
