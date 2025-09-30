import time
import board
import busio
import digitalio
from adafruit_ads1x15.ads1115 import ADS1115
from adafruit_ads1x15.analog_in import AnalogIn

# -----------------------------
# I2C Setup for ADS1115
# -----------------------------
i2c = busio.I2C(board.SCL, board.SDA)  # FT232H SCL/SDA pins
ads = ADS1115(i2c)
ads.gain = 1  # +/- 4.096 V range

# Read channel 0
chan0 = AnalogIn(ads, ADS1115.P0)

# -----------------------------
# GPIO Setup (C0–C3 as example)
# -----------------------------
gpio_pins = {}
for pin_name in ["C0", "C1", "C2", "C3"]:
    pin = digitalio.DigitalInOut(getattr(board, pin_name))
    pin.direction = digitalio.Direction.OUTPUT
    gpio_pins[pin_name] = pin

# Example input pin
input_pin = digitalio.DigitalInOut(board.C4)
input_pin.direction = digitalio.Direction.INPUT
input_pin.pull = digitalio.Pull.UP

# -----------------------------
# Test loop
# -----------------------------
print("Starting FT232H + ADS1115 test...")
while True:
    # Read ADS1115 channel 0
    print(f"ADS1115 CH0 Voltage: {chan0.voltage:.6f} V")

    # Blink GPIO outputs
    for name, pin in gpio_pins.items():
        pin.value = True
        print(f"{name} HIGH")
        time.sleep(0.1)
        pin.value = False
        print(f"{name} LOW")
        time.sleep(0.1)

    # Read GPIO input
    print(f"C4 Input = {input_pin.value}")

    time.sleep(1)
