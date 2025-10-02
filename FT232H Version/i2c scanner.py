import board, busio

i2c = busio.I2C(board.SCL, board.SDA)
print("Scanning I2C bus...")

while not i2c.try_lock():
    pass

try:
    addresses = i2c.scan()
    if addresses:
        print("Found I2C addresses:", [hex(a) for a in addresses])
    else:
        print("No I2C devices found")
finally:
    i2c.unlock()
