import time, math, board, busio
from adafruit_bme280 import basic as adafruit_bme280

# -----------------------
# Psychrometric helpers
# -----------------------
def dewpoint_c(temp_c, rh):
    a, b = 17.62, 243.12
    gamma = (a * temp_c) / (b + temp_c) + math.log(rh/100.0)
    return (b * gamma) / (a - gamma)

def wetbulb_c(temp_c, rh):
    return temp_c*math.atan(0.151977*(rh+8.313659)**0.5) \
           + math.atan(temp_c+rh) \
           - math.atan(rh-1.676331) \
           + 0.00391838*(rh**1.5)*math.atan(0.023101*rh) \
           - 4.686035

def humidity_ratio(temp_c, rh, pressure_hpa=1013.25):
    es = 6.112*math.exp((17.67*temp_c)/(temp_c+243.5))
    e = rh/100.0 * es
    p = pressure_hpa
    return 0.62198*e/(p-e)  # kg/kg

def enthalpy(temp_c, w):
    return 1.006*temp_c + w*(2501 + 1.805*temp_c)  # kJ/kg dry air

# -----------------------
# Sensor sanity check
# -----------------------
i2c = busio.I2C(board.SCL, board.SDA)
sensor = None

while sensor is None:
    while not i2c.try_lock():
        pass
    addresses = [hex(x) for x in i2c.scan()]
    i2c.unlock()

    print("I2C scan:", addresses)

    if "0x76" in addresses:
        sensor = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x76)
        print("✅ BME280 found at 0x76")
    elif "0x77" in addresses:
        sensor = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=0x77)
        print("✅ BME280 found at 0x77")
    else:
        print("❌ No BME280 found, retrying in 3s...")
        time.sleep(3)

# -----------------------
# Main loop: log only
# -----------------------
while True:
    temp_c = sensor.temperature
    rh     = sensor.relative_humidity
    pressure = sensor.pressure
    alt = sensor.altitude

    temp_f = temp_c*9/5+32
    dp_c = dewpoint_c(temp_c, rh)
    dp_f = dp_c*9/5+32
    wb_c = wetbulb_c(temp_c, rh)
    wb_f = wb_c*9/5+32
    w = humidity_ratio(temp_c, rh, pressure)
    h = enthalpy(temp_c, w)

    print("==== HowlX Readings ====")
    print(f"Dry Bulb: {temp_c:.2f} °C / {temp_f:.2f} °F")
    print(f"Dew Point: {dp_c:.2f} °C / {dp_f:.2f} °F")
    print(f"Wet Bulb: {wb_c:.2f} °C / {wb_f:.2f} °F")
    print(f"Humidity: {rh:.2f} %")
    print(f"Pressure: {pressure:.2f} hPa")
    print(f"Altitude: {alt:.2f} m")
    print(f"Humidity Ratio: {w:.5f} kg/kg")
    print(f"Enthalpy: {h:.2f} kJ/kg")
    print("========================\n")

    time.sleep(10)
