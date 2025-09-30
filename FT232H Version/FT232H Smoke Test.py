import time
import board
import digitalio

print("FT232H GPIO blink test...")

# Setup pin C0 as output
led = digitalio.DigitalInOut(board.C0)
led.direction = digitalio.Direction.OUTPUT

while True:
    led.value = True
    print("C0 HIGH")
    time.sleep(0.5)

    led.value = False
    print("C0 LOW")
    time.sleep(0.5)
