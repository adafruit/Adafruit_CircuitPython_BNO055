import time
import board
import busio
import adafruit_bno055
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c)

while True:
    print(sensor.mode) 
    print(sensor.euler)
    print("="*40)
    print("\n")
    time.sleep(.2)
