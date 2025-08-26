# Howlx Atmos Sensor v1.0.1 — HowlX proto board (Feather ESP32-S2 + BME280 + MAX17048)
# - Reads BME280 (0x76/0x77)
# - Reads MAX17048 fuel gauge (0x36)
# - Posts to Adafruit IO Group (batch)
# - Infers charging/discharging/charged and persists last %/V across deep sleep
# - 2025-08-26 v1.0.1: Exception handling, watchdog, retries

import time, math, ssl, struct, supervisor
import board
import busio
import wifi, socketpool
import adafruit_requests
from os import getenv
from adafruit_bme280 import basic as adafruit_bme280
import adafruit_max1704x
import microcontroller, watchdog   # UID + hardware watchdog

# ---- Retry + Watchdog helpers ----
def setup_watchdog(seconds=15.0):
    try:
        microcontroller.watchdog.timeout = seconds
        microcontroller.watchdog.mode = watchdog.WatchDogMode.RESET
        microcontroller.watchdog.feed()
        print(f"WDT armed: {seconds}s")
    except Exception as e:
        print("Watchdog not available:", e)

def feed_wdt():
    try:
        microcontroller.watchdog.feed()
    except Exception:
        pass

def with_retry(fn, tries=4, base_delay=0.5, label=""):
    delay = base_delay
    for attempt in range(1, tries + 1):
        try:
            feed_wdt()
            return fn()
        except Exception as e:
            print(f"{label} attempt {attempt}/{tries} failed: {e}")
            time.sleep(delay)
            delay *= 2
    raise RuntimeError(f"{label} failed after {tries} tries")

# ---------- Settings ----------
SENSOR_NAME = "HowlX Atmos"
WIFI_SSID = getenv("WIFI_SSID")
WIFI_PASSWORD = getenv("WIFI_PASSWORD")
AIO_USER = getenv("ADAFRUIT_AIO_USERNAME")
AIO_KEY  = getenv("ADAFRUIT_AIO_KEY")
AIO_GROUP = getenv("AIO_GROUP_KEY")

SLEEP_SECONDS = 300         # 5 minutes
SEA_LEVEL_HPA = 1013.25     # for better altitude calc

setup_watchdog(15.0)        # <-- ARM WDT EARLY

# --------------- I2C Setup ---------------
i2c = board.I2C()
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

# --------------- Unique ID ---------------
def _uid_bytes():
      """
      Return UID as bytes regardless of how CircuitPython exposes it
      (bytes, list of ints, or string).
      """
    uid = microcontroller.cpu.uid
    if isinstance(uid, (bytes, bytearray)):
        return bytes(uid)
    if isinstance(uid, str):
        # strip common "0x", colons, spaces; interpret as hex
        s = uid.replace(":", "").replace(" ", "").lower()
        if s.startswith("0x"):
            s = s[2:]
        return bytes.fromhex(s)
    # iterable of ints/strings
    return bytes(int(x) for x in uid)

uid_hex = "".join(f"{b:02x}" for b in _uid_bytes())
sensor_id = uid_hex[-6:]  # last 3 bytes, e.g. "a1b2c3"

# --------------- Print Info --------------
print(f"{SENSOR_NAME} #{sensor_id} ready")
print("Temperature: %.1f °C" % bme280.temperature)
print("Humidity: %.1f %%" % bme280.humidity)
print("Pressure: %.1f hPa" % bme280.pressure) 

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

try:
    # ---------- BME280: read BEFORE Wi-Fi ----------
    sensor = None
    while sensor is None:
        while not i2c.try_lock():
            feed_wdt()
        addrs = [hex(x) for x in i2c.scan()]
        i2c.unlock()
        if "0x76" in addrs:
            sensor = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)
        elif "0x77" in addrs:
            sensor = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x77)
        else:
            print("No BME280 found; I2C:", addrs, "retrying in 2s…")
            time.sleep(2)
            feed_wdt()

    sensor.sea_level_pressure = SEA_LEVEL_HPA

    def _read_bme_once():
        return (float(sensor.temperature),
                float(sensor.relative_humidity),
                float(sensor.pressure),
                float(sensor.altitude))

    temp_c, rh, pressure_hpa, alt_m = with_retry(_read_bme_once, label="BME280 read")

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

    # ---------- Battery (MAX17048/49 @ 0x36) ----------
    fg = adafruit_max1704x.MAX17048(i2c)

    def usb_present():
        try:
            return bool(supervisor.runtime.usb_connected)
        except Exception:
            pass
        try:
            if hasattr(microcontroller.cpu, "vbus_present"):
                return bool(microcontroller.cpu.vbus_present)
        except Exception:
            pass
        try:
            import digitalio
            for name in ("VBUS", "VBUS_SENSE", "VUSB", "5V"):
                if hasattr(board, name):
                    pin = digitalio.DigitalInOut(getattr(board, name))
                    pin.switch_to_input()
                    lvl = pin.value
                    pin.deinit()
                    return bool(lvl)
        except Exception:
            pass
        return None

    time.sleep(0.1); v1, p1 = fg.cell_voltage, fg.cell_percent
    time.sleep(0.1); v2, p2 = fg.cell_voltage, fg.cell_percent
    vbat = (v1 + v2) / 2.0
    bpct = (p1 + p2) / 2.0
    usb = usb_present()

    prev_pct = prev_v = None
    try:
        import alarm
        mem = alarm.sleep_memory
        if len(mem) >= 8:
            prev_pct, prev_v = struct.unpack("ff", bytes(mem[:8]))
    except Exception:
        pass

    def infer_state(usb_present_flag, v_now, p_now, p_prev=None, v_prev=None):
        if usb_present_flag is True:
            if v_now >= 4.18 or p_now >= 99.0:
                return "charged"
            if (p_prev is not None and (p_now - p_prev) > 0.05) or \
               (v_prev is not None and (v_now - v_prev) > 0.01):
                return "charging"
            return "usb-present"
        elif usb_present_flag is False:
            return "discharging"
        else:
            if (p_prev is not None and (p_now - p_prev) > 0.05) or \
               (v_prev is not None and (v_now - v_prev) > 0.01):
                return "charging"
            return "discharging"

    charging_state = infer_state(usb, vbat, bpct, prev_pct, prev_v)
    print(f"Battery: {vbat:.3f} V | {bpct:.1f}% | state={charging_state}")

    # ---------- Wi-Fi + HTTP session ----------
    print("Connecting Wi-Fi…")
    def _wifi_connect():
        wifi.radio.connect(WIFI_SSID, WIFI_PASSWORD)
        return True
    with_retry(_wifi_connect, label="Wi-Fi connect")

    pool = socketpool.SocketPool(wifi.radio)
    requests = adafruit_requests.Session(pool, ssl.create_default_context())
    print("Wi-Fi OK:", wifi.radio.ipv4_address)

    # ---------- Adafruit IO Group batch POST ----------
    BASE = f"https://io.adafruit.com/api/v2/{AIO_USER}"
    URL  = f"{BASE}/groups/{AIO_GROUP}/data"
    HEADERS = {"X-AIO-Key": AIO_KEY, "Content-Type": "application/json"}

    feeds_payload = {
        "feeds": [
            # --- Identification ---
            {"key": "sensor-name", "value": f"{SENSOR_NAME} #{sensor_id}"},
            {"key": "sensor-id", "value": sensor_id},   # <-- OPTIONAL: remove if you want name-only

            # --- Environment ---
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

            # --- Battery ---
            {"key": "battery-v",             "value": round(vbat, 3)},
            {"key": "battery-pct",           "value": round(bpct, 1)},
            {"key": "charging-state",        "value": charging_state},
        ]
    }

    def _post_payload():
        r = requests.post(URL, json=feeds_payload, headers=HEADERS, timeout=15)
        sc = r.status_code
        r.close()
        if sc >= 400:
            raise RuntimeError(f"HTTP {sc}")
        print("AIO group POST status:", sc)
        return sc

    with_retry(_post_payload, label="Adafruit IO POST")

    # ---------- Persist latest battery values for next wake ----------
    try:
        import alarm
        alarm.sleep_memory[:8] = struct.pack("ff", float(bpct), float(vbat))
    except Exception:
        pass

except Exception as e:
    print("Top-level error:", e)
    if not supervisor.runtime.usb_connected:
        time.sleep(2)
        supervisor.reload()   # soft reboot; WDT will hard-reset if truly hung
    else:
        raise

# ---------- Deep sleep ----------
try:
    import alarm
    print(f"Sleeping for {SLEEP_SECONDS} s…")
    feed_wdt()
    t = alarm.time.TimeAlarm(monotonic_time=time.monotonic() + SLEEP_SECONDS)
    alarm.exit_and_deep_sleep_until_alarms(t)
except Exception as e:
    print("Deep sleep not available, fallback loop:", e)
    time.sleep(SLEEP_SECONDS)
