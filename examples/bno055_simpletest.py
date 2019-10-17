import time
import board
import busio
import adafruit_bno055

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055(i2c)

a = [adafruit_bno055.ACCONLY_MODE, adafruit_bno055.MAGONLY_MODE, adafruit_bno055.GYRONLY_MODE, adafruit_bno055.ACCMAG_MODE, adafruit_bno055.ACCGYRO_MODE, adafruit_bno055.MAGGYRO_MODE, adafruit_bno055.AMG_MODE, adafruit_bno055.IMUPLUS_MODE, adafruit_bno055.COMPASS_MODE, adafruit_bno055.M4G_MODE, adafruit_bno055.NDOF_FMC_OFF_MODE, adafruit_bno055.NDOF_MODE] 
for i in a:
    print(i)
    #sensor.mode = i
    sensor.mode = adafruit_bno055.AMG_MODE
    print('Temperature: {} degrees C'.format(sensor.temperature))
    print('Accelerometer (m/s^2): {}'.format(sensor.acceleration))
    print('Magnetometer (microteslas): {}'.format(sensor.magnetic))
    print('Gyroscope (rad/sec): {}'.format(sensor.gyro))
    print('Euler angle: {}'.format(sensor.euler))
    print('Quaternion: {}'.format(sensor.quaternion))
    print('Linear acceleration (m/s^2): {}'.format(sensor.linear_acceleration))
    print('Gravity (m/s^2): {}'.format(sensor.gravity))
    print()

    time.sleep(1)
