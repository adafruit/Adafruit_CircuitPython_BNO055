import time
import board
import busio
import adafruit_bno055

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c)

while True:
    print('Temperature: {} degrees C'.format(sensor.temperature))
    print('Accelerometer (m/s^2): {}'.format(sensor.accelerometer))
    print('Magnetometer (microteslas): {}'.format(sensor.magnetometer))
    print('Gyroscope (deg/sec): {}'.format(sensor.gyroscope))
    print('Euler angle: {}'.format(sensor.euler))
    print('Quaternion: {}'.format(sensor.quaternion))
    print('Linear acceleration (m/s^2): {}'.format(sensor.linear_acceleration))
    print('Gravity (m/s^2): {}'.format(sensor.gravity))
    print()

    time.sleep(1)
