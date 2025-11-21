# HowlX Atmos Sensor v1.0.8 — HowlX proto board (Feather ESP32-S2/S3 + BME280/BME680/SHT3x + MAX17048)
# - Reads BME280/SHT3x/BME680 environmental sensors (0x76/0x77/0x44)
# - Reads MAX17048 fuel gauge (0x36)
# - Posts to Adafruit IO Group (batch)
# - Infers charging/discharging/charged and persists last %/V across deep sleep
#
# Version history:
# 2025-08-26 v1.0.1:
#     • Exception handling, watchdog, retries
# 2025-08-26 v1.0.2:
#     • Add calibration support and read for offset file
# 2025-08-26 v1.0.3:
#     • Offset file fallback fetch (GitHub)
# 2025-08-27 v1.0.4:
#     • Add Influx DB Support
# 2025-08-27 v1.0.5:
#     • Correct NameError when no global bme280 is defined
#     • Supports ESP32-S2 boards with no onboard BME280
# 2025-10-20 v1.0.6:
#     • Add BME680 & SHT3x support
#     • Board detection
#     • FW-version tagging
#     • Improved telemetry identification
# 2025-10-21 v1.0.7:
#     • Improved MAX17048 handling (dual-sample average, dual-API quick_start)
#     • USB/VBUS detection and improved charge inference
# 2025-11-21 v1.0.8:
#     • Added VOC/Gas IAQ interpretation layer for BME680 (1–5 index + label)
#     • Added IAQ values to console logs, AIO feeds, and InfluxDB fields
#     • Maintains backward compatibility on boards without gas sensors
# # 2025-11-22 v1.0.9:
#     • Added 10-sample / 10-second gas-resistance averaging for BME680
#     • Added ESPHome-style humidity-compensated VOC metric ("comp_gas")
#     • Added comp_gas to console logs, Adafruit IO feeds, and InfluxDB fields
#     • Improved IAQ printout structure (Gas Ω, IAQ label/index, comp_gas)
#     • Stabilizes VOC readings by delaying air-quality evaluation until after heater warm-up

import time, math, ssl, struct, supervisor, json
import board
import busio
import wifi, socketpool
import adafruit_requests
from os import getenv
from adafruit_bme280 import basic as adafruit_bme280
import adafruit_bme680
import adafruit_sht31d
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

# ---- Influx line protocol helpers ----
def _lp_escape_tag(v):
    # escape commas, spaces, equals in tag values
    return str(v).replace(",", r"\,").replace(" ", r"\ ").replace("=", r"\=")

def _lp_qstr(v):
    # quote string field values for line protocol
    return '"' + str(v).replace('"', r'\"') + '"'
    
# ---- Env helpers (robust across str/int/bool) ----
def env_flag(name, default=False):
    v = getenv(name)
    if v is None or v == "":
        return bool(default)
    if isinstance(v, (bool, int)):
        return bool(v)
    try:
        s = v.decode() if isinstance(v, (bytes, bytearray)) else str(v)
    except Exception:
        s = str(v)
    s = s.strip().strip('"').strip("'").lower()
    return s in ("1", "true", "yes", "on", "y")

def env_int(name, default=0):
    v = getenv(name)
    if v is None or v == "":
        return default
    if isinstance(v, int):
        return v
    try:
        return int(str(v).strip().strip('"').strip("'"))
    except Exception:
        return default

# ---- Calibration helpers ----
def calibrated_flag(off):
    return 1 if (
        abs(off.get("temp", 0.0))  > 0.01 or
        abs(off.get("hum", 0.0))   > 0.01 or
        abs(off.get("press", 0.0)) > 0.01
    ) else 0

def load_offsets():
    # Reads /bme_offsets.json if present; else all zeros
    try:
        with open("/bme_offsets.json", "r") as f:
            d = json.load(f)
            return {"temp": float(d.get("temp", 0.0)),
                    "hum":  float(d.get("hum", 0.0)),
                    "press":float(d.get("press", 0.0))}
    except Exception:
        return {"temp": 0.0, "hum": 0.0, "press": 0.0}


# ---------- Settings ----------
SENSOR_NAME = "HowlX Atmos"
WIFI_SSID = getenv("WIFI_SSID")
WIFI_PASSWORD = getenv("WIFI_PASSWORD")
AIO_USER = getenv("ADAFRUIT_AIO_USERNAME")
AIO_KEY  = getenv("ADAFRUIT_AIO_KEY")
AIO_GROUP = getenv("AIO_GROUP_KEY")

# ---- Feature flags ----
AIO_ENABLE     = env_flag("AIO_ENABLE",  True)   # default ON
INFLUX_ENABLE  = env_flag("INFLUX_ENABLE", False)  # default OFF

# ---- InfluxDB (v2) config ----
INFLUX_URL    = getenv("INFLUX_URL")    or "https://influx.wolfco.io"
INFLUX_ORG    = getenv("INFLUX_ORG")    or ""   # e.g., "wolfco"
INFLUX_BUCKET = getenv("INFLUX_BUCKET") or ""   # e.g., "howlx-atmos"
INFLUX_TOKEN  = getenv("INFLUX_TOKEN")  or ""   # Influx API token

# (optional) Influx v1 compat (leave blank to skip)
INFLUX_V1_DB   = getenv("INFLUX_V1_DB")   or ""
INFLUX_V1_USER = getenv("INFLUX_V1_USER") or ""
INFLUX_V1_PASS = getenv("INFLUX_V1_PASS") or ""

# Operational Variables
SLEEP_SECONDS = 300         # 5 minutes
SEA_LEVEL_HPA = 1013.25     # for better altitude calc

# ---- Calibration config ----
DO_CALIBRATE = False            # True => print JSON offsets and exit
CAL_SECONDS  = 60

# Optional: pull reference values from another board's AIO group
REF_FROM_AIO  = True
AIO_REF_GROUP = getenv("AIO_REF_GROUP") or "howlx-proto-board-002"

# If REF_FROM_AIO = False, set manual reference values:
REF_TEMP_C = None
REF_RH_PCT = None
REF_P_HPA  = None


setup_watchdog(15.0)        # <-- ARM WDT EARLY

# --------------- I2C Setup ---------------
i2c = board.I2C()
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
# --------------- Sensor bring-up (safe, supports BME280/BME680/SHT3x) ---------------
def find_env_sensor(i2c_obj, tries=8, settle_s=0.25):
    """
    Probe for BME680 (0x76/0x77), then BME280 (0x76/0x77), then SHT3x (0x44/0x45)
    Returns (sensor_type, sensor_obj, addr_int, seen_list)
    sensor_type in {"BME680","BME280","SHT3x",None}
    """
    addr = None
    seen = []
    for attempt in range(1, tries + 1):
        # scan
        while not i2c_obj.try_lock():
            feed_wdt()
            time.sleep(0.01)
        try:
            seen = i2c_obj.scan() or []
        finally:
            i2c_obj.unlock()

        # --- Try BME680 ---
        for a in (0x76, 0x77):
            if a in seen:
                try:
                    s = adafruit_bme680.Adafruit_BME680_I2C(i2c_obj, address=a)
                    return "BME680", s, a, seen
                except Exception:
                    pass

        # --- Try BME280 ---
        for a in (0x76, 0x77):
            if a in seen:
                try:
                    s = adafruit_bme280.Adafruit_BME280_I2C(i2c_obj, address=a)
                    return "BME280", s, a, seen
                except Exception:
                    pass

        # --- Try SHT3x ---
        for a in (0x44, 0x45):
            if a in seen:
                try:
                    s = adafruit_sht31d.SHT31D(i2c_obj, address=a)
                    return "SHT3x", s, a, seen
                except Exception:
                    pass

        print("No supported env sensor yet; I2C scan:", [hex(a) for a in seen], f"(retry {attempt}/{tries})")
        time.sleep(settle_s)

    return None, None, addr, seen
# --------------- Detect board model (S2/S3/RP etc.) ---------------
import sys, os
def detect_board_code():
    p = getattr(sys, "platform", "").lower()
    if "esp32s2" in p: return "S2"
    if "esp32s3" in p: return "S3"
    if "rp2040" in p or p == "rp2": return "RP"
    # fallback: uname
    try:
        mach = os.uname().machine.lower()
        if "esp32s2" in mach: return "S2"
        if "esp32s3" in mach: return "S3"
        if "rp2040" in mach: return "RP"
    except Exception:
        pass
    return "UNK"

board_code = detect_board_code()

sensor_type, _sensor_boot, _addr, _seen = find_env_sensor(i2c)

print(f"{SENSOR_NAME} [{sensor_type}:{board_code}-{sensor_id}] ready")


if _sensor_boot:
    print(f"{sensor_type} found at 0x{_addr:02X}")
    if sensor_type == "BME680":
        print("Temperature: %.1f °C" % _sensor_boot.temperature)
        print("Humidity: %.1f %%" % _sensor_boot.humidity)
        print("Pressure: %.1f hPa" % _sensor_boot.pressure)
        print("Gas: %.0f Ω" % _sensor_boot.gas)
    elif sensor_type == "BME280":
        print("Temperature: %.1f °C" % _sensor_boot.temperature)
        print("Humidity: %.1f %%" % _sensor_boot.humidity)
        print("Pressure: %.1f hPa" % _sensor_boot.pressure)
    else:  # SHT3x
        print("Temperature: %.1f °C" % _sensor_boot.temperature)
        print("Humidity: %.1f %%" % _sensor_boot.relative_humidity)
else:
    print("No supported sensor detected (BME680/BME280 @0x76/0x77 or SHT3x @0x44/0x45).")
    print("I2C scan saw:", [("0x%02X" % a) for a in (_seen or [])])


# --- Proposed calibration offsets /bme_offsets.json (if present) ---
try:
    with open("/bme_offsets.json", "r") as f:
        _raw = f.read()
    print("Offsets JSON (/bme_offsets.json):", _raw)
    _offsets_preview = json.loads(_raw)  # {"temp":..., "hum":..., "press":...}
except Exception:
    print("Offsets JSON (/bme_offsets.json): <missing or unreadable>")
    _offsets_preview = {"temp": 0.0, "hum": 0.0, "press": 0.0}

# Use the safely-initialized sensor (_sensor_boot) if present
if _sensor_boot:
    try:
        t_off = float(_offsets_preview.get("temp", 0.0))
        h_off = float(_offsets_preview.get("hum", 0.0))
        p_off = float(_offsets_preview.get("press", 0.0))
    except Exception:
        t_off = h_off = p_off = 0.0

    if sensor_type == "SHT3x":
        temp_preview = (_sensor_boot.temperature + t_off)
        hum_preview  = (_sensor_boot.relative_humidity + h_off)
        print("→ Corrected preview: T=%.2f °C | RH=%.2f %% | P=n/a" % (temp_preview, hum_preview))
    else:
        temp_preview = (_sensor_boot.temperature + t_off)
        hum_preview  = (_sensor_boot.humidity + h_off)
        press_preview = (_sensor_boot.pressure + p_off)
        print("→ Corrected preview: T=%.2f °C | RH=%.2f %% | P=%.2f hPa" % (temp_preview, hum_preview, press_preview))
else:
    print("→ Corrected preview: (no sensor detected yet)")

   
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

# ---------- Gas / IAQ helpers ----------
def interpret_gas_resistance(gas_ohms):
    """
    Very lightweight VOC / air-quality heuristic based only on gas resistance.
    Returns (index, label) where index is 1..5 (1 = bad, 5 = very clean).

    This is NOT Bosch BSEC, just a rough trend indicator:
      >15kΩ  => 5  "very clean"
      8k–15k => 4  "clean"
      3k–8k  => 3  "light VOCs"
      1k–3k  => 2  "moderate VOCs"
      <1k    => 1  "high VOCs"
    """
    if gas_ohms is None:
        return None, None
    try:
        g = float(gas_ohms)
    except Exception:
        return None, None
    if g <= 0:
        return None, None

    if g >= 15000:
        return 5, "very clean"
    elif g >= 8000:
        return 4, "clean"
    elif g >= 3000:
        return 3, "light VOCs"
    elif g >= 1000:
        return 2, "moderate VOCs"
    else:
        return 1, "high VOCs"


try:
    # ---------- Unified read (before Wi-Fi) ----------
    if not _sensor_boot:
        # keep trying until found (like your original)
        while not _sensor_boot:
            sensor_type, _sensor_boot, _addr, _seen = find_env_sensor(i2c, tries=1)
            if not _sensor_boot:
                print("No env sensor found; I2C:", [hex(x) for x in (_seen or [])], "retrying in 2s…")
                time.sleep(2)
                feed_wdt()

    # Configure per-sensor (e.g., sea level for pressure sensors)
    if sensor_type in ("BME280", "BME680"):
        try:
            _sensor_boot.sea_level_pressure = SEA_LEVEL_HPA
        except Exception:
            pass

    def _read_env_once():
        if sensor_type == "BME680":
            return (
                float(_sensor_boot.temperature),
                float(_sensor_boot.humidity),
                float(_sensor_boot.pressure),
                float(getattr(_sensor_boot, "altitude", 0.0)),  # some builds expose altitude
                float(_sensor_boot.gas),
            )
        elif sensor_type == "BME280":
            return (
                float(_sensor_boot.temperature),
                float(_sensor_boot.humidity),
                float(_sensor_boot.pressure),
                float(_sensor_boot.altitude),
                None,  # no gas
            )
        else:  # SHT3x
            return (
                float(_sensor_boot.temperature),
                float(_sensor_boot.relative_humidity),
                None,   # no pressure
                None,   # no altitude
                None,   # no gas
            )

    temp_c, rh, pressure_hpa, alt_m, gas_ohms = with_retry(_read_env_once, label=f"{sensor_type} read")
    # --- IAQ GAS AVERAGING: 10 samples over 10 seconds ---
    def sample_gas_iaq(sensor, samples=10, delay_s=1.0):
        gas_vals = []
        for i in range(samples):
            try:
                g = float(sensor.gas)
                gas_vals.append(g)
                print(f"[IAQ] sample {i+1}/{samples} = {g:.0f} Ω")
            except Exception as e:
                print("Gas read failed:", e)
            time.sleep(delay_s)
            feed_wdt()  # keep WDT alive
        if not gas_vals:
            return None
        avg = sum(gas_vals) / len(gas_vals)
        print(f"[IAQ] 10-second average = {avg:.0f} Ω")
        return avg

    # average gas measurement for IAQ
    if sensor_type == "BME680":
        gas_avg = sample_gas_iaq(_sensor_boot, samples=10, delay_s=1.0)
        if gas_avg is not None:
            gas_ohms = gas_avg
    
    # Simple IAQ interpretation from gas resistance
    iaq_index, iaq_label = interpret_gas_resistance(gas_ohms)
    # ----- ESPHome-style humidity-compensated IAQ metric -----
    import math
    def compute_comp_gas(gas_ohms, rh):
        """
        comp_gas = log(R_gas) + 0.04 * RH
        Higher = cleaner air. Lower = more VOC.
        """
        if gas_ohms is None or gas_ohms <= 0:
            return None
        try:
            return math.log(gas_ohms) + 0.04 * rh
        except Exception:
            return None
    

    # ---- Offsets: load & stash raw readings ----
    t_raw = float(temp_c)
    rh_raw = float(rh)
    p_raw  = None if pressure_hpa is None else float(pressure_hpa)

    offsets = load_offsets()   # {"temp": ..., "hum": ..., "press": ...}

    # ---------- Apply offsets & compute metrics ----------
    t_c = t_raw + offsets.get("temp", 0.0)
    rh  = rh_raw + offsets.get("hum", 0.0)
    p_h = None if p_raw is None else (p_raw + offsets.get("press", 0.0))

    # Psychrometrics: if no pressure, use sea-level for derived calcs only
    p_for_calc = p_h if p_h is not None else SEA_LEVEL_HPA
    # Compute ESPHome-style IAQ compensation
    comp_gas = compute_comp_gas(gas_ohms, rh)

    temp_f = t_c * 9/5 + 32
    dp_c   = dewpoint_c(t_c, rh)
    dp_f   = dp_c * 9/5 + 32
    wb_c   = wetbulb_c(t_c, rh)
    wb_f   = wb_c * 9/5 + 32
    w      = humidity_ratio(t_c, rh, p_for_calc)
    h      = enthalpy(t_c, w)

    print("==== HowlX Atmos Readings ====")
    print(f"Sensor: {sensor_type} @ 0x{_addr:02X}")
    print(f"Dry Bulb: {t_c:.2f} °C / {temp_f:.2f} °F")
    print(f"Dew Point: {dp_c:.2f} °C / {dp_f:.2f} °F")
    print(f"Wet Bulb: {wb_c:.2f} °C / {wb_f:.2f} °F")
    print(f"Humidity: {rh:.2f} %")
    if p_h is not None:
        print(f"Pressure: {p_h:.2f} hPa")
        if alt_m is not None:
            print(f"Altitude: {alt_m:.2f} m")
    else:
        print("Pressure: n/a (no pressure sensor)")
    if gas_ohms is not None:
        print(f"Gas: {gas_ohms:.0f} Ω")
        if iaq_index is not None:
            print(f"Air Quality: {iaq_label} (level {iaq_index}/5)")
    if comp_gas is not None:
        print(f"IAQ (comp_gas): {comp_gas:.2f}")
    print(f"Humidity Ratio: {w:.5f} kg/kg")
    print(f"Enthalpy: {h:.2f} kJ/kg")
    print("========================")

    # ---------- Battery (MAX17048/49 @ 0x36) ----------
    fg = adafruit_max1704x.MAX17048(i2c)

    def usb_present():
        """Detect if USB power is present."""
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

    # First read (average two samples)
    time.sleep(0.1); v1, p1 = fg.cell_voltage, fg.cell_percent
    time.sleep(0.1); v2, p2 = fg.cell_voltage, fg.cell_percent
    vbat = (v1 + v2) / 2.0
    bpct = (p1 + p2) / 2.0
    usb  = usb_present()

    # Kick the gauge if SOC looks wrong, then clamp
    if bpct > 101.5 or bpct < -0.5:
        try:
            # Support both library styles: method or property
            if callable(getattr(fg, "quick_start", None)):
                fg.quick_start()        # method
            else:
                fg.quick_start = True   # property

            time.sleep(0.25)
            # re-read after quick_start
            v1, p1 = fg.cell_voltage, fg.cell_percent
            time.sleep(0.1)
            v2, p2 = fg.cell_voltage, fg.cell_percent
            vbat = (v1 + v2) / 2.0
            bpct = (p1 + p2) / 2.0
            print("MAX17048 quick_start applied")
        except Exception as e:
            print("MAX17048 quick_start failed:", e)

    bpct = max(0.0, min(100.0, bpct))  # clamp to 0..100

    # Load previous values from deep sleep to infer trend
    prev_pct = prev_v = None
    try:
        import alarm
        mem = alarm.sleep_memory
        if len(mem) >= 8:
            prev_pct, prev_v = struct.unpack("ff", bytes(mem[:8]))
    except Exception:
        pass

    def infer_state(usb_present_flag, v_now, p_now, p_prev=None, v_prev=None):
        near_full_v = 4.18
        near_full_p = 99.0
        rising_pct = (p_prev is not None) and ((p_now - p_prev) > 0.10)   # >0.10 %
        rising_v   = (v_prev is not None) and ((v_now - v_prev) > 0.008)  # >8 mV

        if usb_present_flag is True:
            if (v_now >= near_full_v) or (p_now >= near_full_p):
                return "charged"
            if rising_pct or rising_v:
                return "charging"
            return "plugged"  # on USB but not clearly charging/discharging

        if usb_present_flag is False:
            return "discharging"

        # USB unknown
        if (v_now >= near_full_v) or (p_now >= near_full_p):
            return "charged"
        if rising_pct or rising_v:
            return "charging"
        return "discharging"

    charging_state = infer_state(usb, vbat, bpct, prev_pct, prev_v)
    print(f"Battery: {vbat:.3f} V | {bpct:.1f}% | state={charging_state}")

    # Persist latest values for next wake
    try:
        import alarm
        alarm.sleep_memory[:8] = struct.pack("ff", float(bpct), float(vbat))
    except Exception:
        pass

    # ---------- Wi-Fi + HTTP session ----------
    print("Connecting Wi-Fi…")
    def _wifi_connect():
        wifi.radio.connect(WIFI_SSID, WIFI_PASSWORD)
        return True
    with_retry(_wifi_connect, label="Wi-Fi connect")

    pool = socketpool.SocketPool(wifi.radio)
    requests = adafruit_requests.Session(pool, ssl.create_default_context())
    print("Wi-Fi OK:", wifi.radio.ipv4_address)
    
    # ---- InfluxDB write (v2 or v1) ----
    def post_to_influx():
        if not INFLUX_ENABLE:
            return 204  # disabled -> pretend OK

        # measurement + tags
        meas = "howlx"
        tags = "device=%s,name=%s" % (_lp_escape_tag(sensor_id), _lp_escape_tag(SENSOR_NAME))

        # fields (use adjusted values you already computed)
        fields = []
        fields.append("temperature_c=%.2f" % (t_c,))
        fields.append("temperature_f=%.2f" % (temp_f,))
        fields.append("dewpoint_c=%.2f" % (dp_c,))
        fields.append("dewpoint_f=%.2f" % (dp_f,))
        fields.append("wetbulb_c=%.2f" % (wb_c,))
        fields.append("wetbulb_f=%.2f" % (wb_f,))
        fields.append("humidity_pct=%.2f" % (rh,))
        fields.append("humidity_ratio_kgkg=%.5f" % (w,))
        fields.append("enthalpy_kjkg=%.2f" % (h,))

        # only include these if they exist for the current sensor
        if p_h is not None:
            fields.append("pressure_hpa=%.2f" % (p_h,))
            if alt_m is not None:
                fields.append("altitude_m=%.2f" % (alt_m,))
        if gas_ohms is not None:
            fields.append("gas_ohms=%di" % int(gas_ohms))
            if iaq_index is not None:
                fields.append("gas_iaq_index=%di" % int(iaq_index))
                fields.append("gas_iaq_label=%s" % _lp_qstr(iaq_label))
                if comp_gas is not None:
                    fields.append("gas_iaq_comp=%.2f" % float(comp_gas))
                

        fields.append("battery_v=%.3f" % (vbat,))
        fields.append("battery_pct=%.1f" % (bpct,))
        fields.append('charging_state=%s' % _lp_qstr(charging_state))
        fields.append("offset_temp=%.2f" % (offsets.get("temp", 0.0),))
        fields.append("offset_hum=%.2f" % (offsets.get("hum", 0.0),))
        if p_raw is not None:
            fields.append("offset_press=%.2f" % (offsets.get("press", 0.0),))
        fields.append("calibrated=%di" % (1 if calibrated_flag(offsets) else 0))

        line = "%s,%s %s" % (meas, tags, ",".join(fields))
        # no timestamp -> server receive time

        # Prefer v2
        if INFLUX_TOKEN and INFLUX_ORG and INFLUX_BUCKET:
            url = "%s/api/v2/write?org=%s&bucket=%s&precision=s" % (INFLUX_URL, INFLUX_ORG, INFLUX_BUCKET)
            headers = {
                "Authorization": "Token %s" % INFLUX_TOKEN,
                "Content-Type": "text/plain; charset=utf-8",
                "Accept": "application/json",
            }
            r = requests.post(url, data=line + "\n", headers=headers, timeout=15)
            sc = r.status_code
            r.close()
            if sc >= 400:
                raise RuntimeError("Influx v2 HTTP %d" % sc)
            print("Influx v2 write status:", sc)
            return sc

        # Fallback v1
        if INFLUX_V1_DB:
            auth_q = ""
            if INFLUX_V1_USER and INFLUX_V1_PASS:
                auth_q = "&u=%s&p=%s" % (INFLUX_V1_USER, INFLUX_V1_PASS)
            url = "%s/write?db=%s&precision=s%s" % (INFLUX_URL, INFLUX_V1_DB, auth_q)
            r = requests.post(url, data=line + "\n", headers={"Content-Type": "text/plain"}, timeout=15)
            sc = r.status_code
            r.close()
            if sc >= 400:
                raise RuntimeError("Influx v1 HTTP %d" % sc)
            print("Influx v1 write status:", sc)
            return sc

        print("Influx not configured (missing org/bucket/token or v1 db)")
        return 204
    
    # ---- Offset file fallback fetch (GitHub) ----
    OFFSETS_GH_URL = "https://raw.githubusercontent.com/FangWolf96/howlx-temp/refs/heads/main/bme_offsets.json"

    def _offsets_file_exists():
        try:
            import os
            return "bme_offsets.json" in os.listdir("/")
        except Exception:
            return False

    # Set via env INTEGRATED_SENSOR=1 (default 1). Set to 0 to skip remote fetch on this board.
    INTEGRATED_SENSOR = (getenv("INTEGRATED_SENSOR") or "0") == "1"  # default disabled; set to "1" to enable

    if (not DO_CALIBRATE) and INTEGRATED_SENSOR and (not _offsets_file_exists()):
        try:
            print("Offsets: local file missing; fetching from GitHub...")
            r = requests.get(OFFSETS_GH_URL, timeout=15)
            text = r.text
        except Exception as e:
            print("Offsets: GET failed:", e)
        else:
            try:
                gh_offsets = json.loads(text)
                gh_offsets = {
                    "temp": float(gh_offsets.get("temp", 0.0)),
                    "hum":  float(gh_offsets.get("hum", 0.0)),
                    "press":float(gh_offsets.get("press", 0.0)),
                }
                offsets = gh_offsets  # use for this run

                # Persist for next boots
                try:
                    with open("/bme_offsets.json", "w") as f:
                        f.write(json.dumps(gh_offsets))
                    print("Offsets: saved to /bme_offsets.json")
                except Exception as e:
                    print("Offsets: save failed:", e)

                # Re-apply offsets to current-cycle readings
                t_c = t_raw + offsets["temp"]
                rh  = rh_raw + offsets["hum"]
                p_h = p_raw  + offsets["press"]
                temp_f = t_c * 9/5 + 32
                dp_c   = dewpoint_c(t_c, rh)
                dp_f   = dp_c * 9/5 + 32
                wb_c   = wetbulb_c(t_c, rh)
                wb_f   = wb_c * 9/5 + 32
                w      = humidity_ratio(t_c, rh, p_h)
                h      = enthalpy(t_c, w)
                # If you want the rest of the script to treat adjusted values as "raw":
                # temp_c = t_c; pressure_hpa = p_h
            except Exception as e:
                print("Offsets: parse/apply failed:", e)
            finally:
                try:
                    r.close()
                except Exception:
                    pass

    # ---- Remote AIO reference helper ----
    def _get_ref_from_aio_group():
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
    
    # ---- Calibration mode (print JSON; no file writes) ----
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
            refT, refH, refP = _get_ref_from_aio_group()
            print("Reference from AIO:", refT, refH, refP)
            if None in (refT, refH, refP):
                raise RuntimeError("Ref from AIO missing (temp/hum/press). Check AIO_REF_GROUP & feeds.")
        else:
            refT, refH, refP = REF_TEMP_C, REF_RH_PCT, REF_P_HPA
            if None in (refT, refH, refP):
                raise RuntimeError("Set REF_* values for manual calibration.")

        new_offsets = {
            "temp": round(refT - mean_T, 2),
            "hum":  round(refH - mean_H, 2),
            "press":round(refP - mean_P, 2),
        }
        print(">>> Copy this JSON into /bme_offsets.json on CIRCUITPY:")
        print(json.dumps(new_offsets))
        raise SystemExit
    
 # ---------- Adafruit IO Group batch POST ----------
    if AIO_ENABLE:
        BASE = f"https://io.adafruit.com/api/v2/{AIO_USER}"
        URL  = f"{BASE}/groups/{AIO_GROUP}/data"
        HEADERS = {"X-AIO-Key": AIO_KEY, "Content-Type": "application/json"}

        # BEGIN REPLACE: build feeds conditionally per sensor
        feeds = [
            # --- Identification ---
            {"key": "sensor-name", "value": f"{SENSOR_NAME} #{sensor_id}"},
            {"key": "sensor-id",   "value": sensor_id},
            {"key": "sensor-type", "value": sensor_type},
            {"key": "fw-version",  "value": "1.0.7"},

            # --- Environment (corrected) ---
            {"key": "temperature-c",         "value": round(t_c, 2)},
            {"key": "temperature-f",         "value": round(temp_f, 2)},
            {"key": "dewpoint-c",            "value": round(dp_c, 2)},
            {"key": "dewpoint-f",            "value": round(dp_f, 2)},
            {"key": "wetbulb-c",             "value": round(wb_c, 2)},
            {"key": "wetbulb-f",             "value": round(wb_f, 2)},
            {"key": "humidity-pct",          "value": round(rh, 2)},
            {"key": "humidity-ratio-kgkg",   "value": round(w, 5)},
            {"key": "enthalpy-kjkg",         "value": round(h, 2)},
        ]

        if p_h is not None:
            feeds.append({"key": "pressure-hpa", "value": round(p_h, 2)})
            if alt_m is not None:
                feeds.append({"key": "altitude-m", "value": round(alt_m, 2)})

        if gas_ohms is not None:
            feeds.append({"key": "gas-ohms", "value": int(gas_ohms)})
            if iaq_index is not None:
                feeds.append({"key": "gas-iaq-index", "value": int(iaq_index)})
                feeds.append({"key": "gas-iaq-label", "value": iaq_label})
                if comp_gas is not None:
                    feeds.append({"key": "gas-iaq-comp", "value": round(comp_gas, 2)})
                

        # --- Battery ---
        feeds += [
            {"key": "battery-v",             "value": round(vbat, 3)},
            {"key": "battery-pct",           "value": round(bpct, 1)},
            {"key": "charging-state",        "value": charging_state},
        ]

        # --- Calibration info ---
        feeds += [
            {"key": "offset-temp",  "value": round(offsets.get("temp", 0.0), 2)},
            {"key": "offset-hum",   "value": round(offsets.get("hum", 0.0), 2)},
        ]
        if p_raw is not None:
            feeds.append({"key": "offset-press", "value": round(offsets.get("press", 0.0), 2)})
        feeds.append({"key": "calibrated", "value": calibrated_flag(offsets)})

        feeds_payload = {"feeds": feeds}
        # END REPLACE

        def _post_payload():
            r = requests.post(URL, json=feeds_payload, headers=HEADERS, timeout=15)
            sc = r.status_code
            r.close()
            if sc >= 400:
                raise RuntimeError(f"HTTP {sc}")
            print("AIO group POST status:", sc)
            return sc
            
        with_retry(_post_payload, label="Adafruit IO POST")
    else:
        print("AIO disabled by settings.")
        
    # --- InfluxDB write (guarded) ---
    if INFLUX_ENABLE:
        try:
            with_retry(post_to_influx, label="Influx write")
        except Exception as e:
            print("Influx write failed:", e)
    else:
        print("Influx disabled by settings.")


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
