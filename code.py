import time
import board
import busio
from adafruit_bme280 import basic as adafruit_bme280

# Create I2C once
i2c = busio.I2C(board.SCL, board.SDA)

def wait_for_bme_address(i2c):
    """Continuously scan I2C until 0x76 or 0x77 is found; return the address."""
    print("Scanning for BME280 on I2C (looking for 0x76 or 0x77)...")
    while True:
        # lock, scan, unlock
        while not i2c.try_lock():
            pass
        addrs = i2c.scan()
        i2c.unlock()

        if addrs:
            # Print once in a while to show progress
            print("I2C addresses:", [hex(x) for x in addrs])

        # Prefer 0x76 if both present; otherwise pick whichever is present
        if 0x76 in addrs:
            return 0x76
        if 0x77 in addrs:
            return 0x77

        # Not found yet; wait a bit and try again
        time.sleep(0.5)

def init_bme(i2c, address):
    """Try to initialize the BME280 at the provided address, retrying until it succeeds."""
    while True:
        try:
            sensor = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=address)
            return sensor
        except ValueError:
            # Could be a transient; re-check address in case wiring changed
            print("Found expected address but init failed; rechecking bus...")
            time.sleep(0.5)
            addr = wait_for_bme_address(i2c)
            address = addr  # update and try again

def main():
    addr = wait_for_bme_address(i2c)
    print("BME280 detected at:", hex(addr))

    bme = init_bme(i2c, addr)

    # Set your local sea-level pressure (hPa) for better altitude calculation
    bme.sea_level_pressure = 1013.25

    while True:
        c = bme.temperature
        f = c * 9.0/5.0 + 32.0
        rh = bme.humidity
        p = bme.pressure
        alt = bme.altitude

        print("Temp: {:.2f} °C / {:.2f} °F".format(c, f))
        print("Humidity: {:.2f} %".format(rh))
        print("Pressure: {:.2f} hPa".format(p))
        print("Altitude: {:.2f} m".format(alt))
        print("-----------------------------")
        time.sleep(2)

if __name__ == "__main__":
    main()
