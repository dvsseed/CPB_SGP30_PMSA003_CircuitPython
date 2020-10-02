""" Example for using the SGP30 with CircuitPython and the Adafruit library"""

import time
import busio
import adafruit_sgp30
import board
import neopixel
import adafruit_pm25

pixels = neopixel.NeoPixel(board.NEOPIXEL, 10, brightness=0.2, auto_write=False)
i2c_bus = busio.I2C(board.SCL, board.SDA, frequency=100000)

def color_chase_all(color, wait):
    for i in range(10):
        pixels[i] = color
        time.sleep(wait)
        pixels.show()
    time.sleep(0.5)

def color_chase_1(color, idx, wait):
    pixels[idx] = color
    time.sleep(wait)
    pixels.show()
    time.sleep(0.1)

RED = (255, 0, 0)
GREEN = (0, 255, 0)
OFF = (0, 0, 0)

# Create library object on I2C port
sgp30 = adafruit_sgp30.Adafruit_SGP30(i2c_bus)

print("SGP30 serial #", [hex(i) for i in sgp30.serial])

sgp30.iaq_init()
# co2eq_base, tvoc_base = sgp30.baseline_eCO2, sgp30.baseline_TVOC
# sgp30.set_iaq_baseline(co2eq_base, tvoc_base)

elapsed_sec = 0
reset_pin = None
uart = busio.UART(board.TX, board.RX, baudrate=9600)
pm25 = adafruit_pm25.PM25_UART(uart, reset_pin)

while True:
    print("eCO2 = %d ppm \t TVOC = %d ppb" % (sgp30.eCO2, sgp30.TVOC))
    print("Raw H2 = %d \t Raw Ethanol = %d" % (sgp30.H2, sgp30.Ethanol))

    color_chase_1(OFF, 0, 0.1)
    if sgp30.TVOC < 10:
        color_chase_1(RED, 0, 0.1)  # Increase the number to slow down the color chase
    else:
        color_chase_1(GREEN, 0, 0.1)

    time.sleep(1)
    elapsed_sec += 1
    if elapsed_sec > 10:
        elapsed_sec = 0
        print(
            "**** Baseline values: eCO2 = 0x%x, TVOC = 0x%x"
            % (sgp30.baseline_eCO2, sgp30.baseline_TVOC)
        )

    try:
        aqdata = pm25.read()
        # print(aqdata)
    except RuntimeError:
        print("Unable to read from sensor, retrying...")
        continue

    print()
    print("Concentration Units (standard)")
    print("---------------------------------------")
    print(
        "PM 1.0: %d\tPM2.5: %d\tPM10: %d"
        % (aqdata["pm10 standard"], aqdata["pm25 standard"], aqdata["pm100 standard"])
    )
    print("Concentration Units (environmental)")
    print("---------------------------------------")
    print(
        "PM 1.0: %d\tPM2.5: %d\tPM10: %d"
        % (aqdata["pm10 env"], aqdata["pm25 env"], aqdata["pm100 env"])
    )
    print("---------------------------------------")
    print("Particles > 0.3um / 0.1L air:", aqdata["particles 03um"])
    print("Particles > 0.5um / 0.1L air:", aqdata["particles 05um"])
    print("Particles > 1.0um / 0.1L air:", aqdata["particles 10um"])
    print("Particles > 2.5um / 0.1L air:", aqdata["particles 25um"])
    print("Particles > 5.0um / 0.1L air:", aqdata["particles 50um"])
    print("Particles > 10 um / 0.1L air:", aqdata["particles 100um"])
    print("---------------------------------------")

    color_chase_1(OFF, 9, 0.1)
    if aqdata["pm25 standard"] < 50:
        color_chase_1(RED, 9, 0.1)
    else:
        color_chase_1(GREEN, 9, 0.1)