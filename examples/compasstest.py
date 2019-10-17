import time
import math
import board
import busio
import adafruit_bno055
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c)

while True:
    x, y, z = sensor.magnetometer
    # convert to gauss
    heading = (math.atan2(y, x) * 180) / math.pi

    if heading < 0:
        heading = 360 + heading

    elif heading > 360:
        heading = heading - 360

    print(heading)
