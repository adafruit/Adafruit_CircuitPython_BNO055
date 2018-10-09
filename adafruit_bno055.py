# The MIT License (MIT)
#
# Copyright (c) 2017 Radomir Dopieralski for Adafruit Industries.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


"""
``adafruit_bno055`` - Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055
=======================================================================================

This is a CircuitPython driver for the Bosch BNO055 nine degree of freedom
inertial measurement unit module with sensor fusion.

* Author(s): Radomir Dopieralski
"""
import time

from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_struct import Struct, UnaryStruct

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_BNO055.git"

_CHIP_ID = const(0xa0)

CONFIG_MODE = const(0x00)
ACCONLY_MODE = const(0x01)
MAGONLY_MODE = const(0x02)
GYRONLY_MODE = const(0x03)
ACCMAG_MODE = const(0x04)
ACCGYRO_MODE = const(0x05)
MAGGYRO_MODE = const(0x06)
AMG_MODE = const(0x07)
IMUPLUS_MODE = const(0x08)
COMPASS_MODE = const(0x09)
M4G_MODE = const(0x0a)
NDOF_FMC_OFF_MODE = const(0x0b)
NDOF_MODE = const(0x0c)

_POWER_NORMAL = const(0x00)
_POWER_LOW = const(0x01)
_POWER_SUSPEND = const(0x02)

_MODE_REGISTER = const(0x3d)
_PAGE_REGISTER = const(0x07)
_CALIBRATION_REGISTER = const(0x35)
_TRIGGER_REGISTER = const(0x3f)
_POWER_REGISTER = const(0x3e)
_ID_REGISTER = const(0x00)


class _ScaledReadOnlyStruct(Struct): # pylint: disable=too-few-public-methods
    def __init__(self, register_address, struct_format, scale):
        super(_ScaledReadOnlyStruct, self).__init__(
            register_address, struct_format)
        self.scale = scale

    def __get__(self, obj, objtype=None):
        result = super(_ScaledReadOnlyStruct, self).__get__(obj, objtype)
        return tuple(self.scale * v for v in result)

    def __set__(self, obj, value):
        raise NotImplementedError()


class _ReadOnlyUnaryStruct(UnaryStruct): # pylint: disable=too-few-public-methods
    def __set__(self, obj, value):
        raise NotImplementedError()


class BNO055:
    """
    Driver for the BNO055 9DOF IMU sensor.
    """

    temperature = _ReadOnlyUnaryStruct(0x34, 'B')
    """Measures the temperature of the chip in degrees Celsius."""
    accelerometer = _ScaledReadOnlyStruct(0x08, '<hhh', 1/100)
    """Gives the raw accelerometer readings, in m/s.

       .. warning:: This is deprecated. Use ``acceleration`` instead. It'll work
         with other drivers too."""
    acceleration = _ScaledReadOnlyStruct(0x08, '<hhh', 1/100)
    """Gives the raw accelerometer readings, in m/s."""
    magnetometer = _ScaledReadOnlyStruct(0x0e, '<hhh', 1/16)
    """Gives the raw magnetometer readings in microteslas.

       .. warning:: This is deprecated. Use ``magnetic`` instead. It'll work with
         other drivers too."""
    magnetic = _ScaledReadOnlyStruct(0x0e, '<hhh', 1/16)
    """Gives the raw magnetometer readings in microteslas."""
    gyroscope = _ScaledReadOnlyStruct(0x14, '<hhh', 1/900)
    """Gives the raw gyroscope reading in degrees per second."""
    euler = _ScaledReadOnlyStruct(0x1a, '<hhh', 1/16)
    """Gives the calculated orientation angles, in degrees."""
    quaternion = _ScaledReadOnlyStruct(0x20, '<hhhh', 1/(1<<14))
    """Gives the calculated orientation as a quaternion."""
    linear_acceleration = _ScaledReadOnlyStruct(0x28, '<hhh', 1/100)
    """Returns the linear acceleration, without gravity, in m/s."""
    gravity = _ScaledReadOnlyStruct(0x2e, '<hhh', 1/100)
    """Returns the gravity vector, without acceleration in m/s."""

    def __init__(self, i2c, address=0x28):
        self.i2c_device = I2CDevice(i2c, address)
        self.buffer = bytearray(2)
        chip_id = self._read_register(_ID_REGISTER)
        if chip_id != _CHIP_ID:
            raise RuntimeError("bad chip id (%x != %x)" % (chip_id, _CHIP_ID))
        self.reset()
        self._write_register(_POWER_REGISTER, _POWER_NORMAL)
        self._write_register(_PAGE_REGISTER, 0x00)
        self._write_register(_TRIGGER_REGISTER, 0x00)
        time.sleep(0.01)
        self.mode = NDOF_MODE
        time.sleep(0.01)

    def _write_register(self, register, value):
        self.buffer[0] = register
        self.buffer[1] = value
        with self.i2c_device as i2c:
            i2c.write(self.buffer)

    def _read_register(self, register):
        self.buffer[0] = register
        with self.i2c_device as i2c:
            i2c.write(self.buffer, end=1, stop=False)
            i2c.readinto(self.buffer, start=1)
        return self.buffer[1]

    def reset(self):
        """Resets the sensor to default settings."""
        self.mode = CONFIG_MODE
        try:
            self._write_register(_TRIGGER_REGISTER, 0x20)
        except OSError: # error due to the chip resetting
            pass
        # wait for the chip to reset (650 ms typ.)
        time.sleep(0.7)

    @property
    def mode(self):
        """
        Switch the mode of operation and return the previous mode.

        Mode of operation defines which sensors are enabled and whether the
        measurements are absolute or relative:

        +------------------+-------+---------+------+----------+
        | Mode             | Accel | Compass | Gyro | Absolute |
        +==================+=======+=========+======+==========+
        | CONFIG_MODE      |   -   |   -     |  -   |     -    |
        +------------------+-------+---------+------+----------+
        | ACCONLY_MODE     |   X   |   -     |  -   |     -    |
        +------------------+-------+---------+------+----------+
        | MAGONLY_MODE     |   -   |   X     |  -   |     -    |
        +------------------+-------+---------+------+----------+
        | GYRONLY_MODE     |   -   |   -     |  X   |     -    |
        +------------------+-------+---------+------+----------+
        | ACCMAG_MODE      |   X   |   X     |  -   |     -    |
        +------------------+-------+---------+------+----------+
        | ACCGYRO_MODE     |   X   |   -     |  X   |     -    |
        +------------------+-------+---------+------+----------+
        | MAGGYRO_MODE     |   -   |   X     |  X   |     -    |
        +------------------+-------+---------+------+----------+
        | AMG_MODE         |   X   |   X     |  X   |     -    |
        +------------------+-------+---------+------+----------+
        | IMUPLUS_MODE     |   X   |   -     |  X   |     -    |
        +------------------+-------+---------+------+----------+
        | COMPASS_MODE     |   X   |   X     |  -   |     X    |
        +------------------+-------+---------+------+----------+
        | M4G_MODE         |   X   |   X     |  -   |     -    |
        +------------------+-------+---------+------+----------+
        | NDOF_FMC_OFF_MODE|   X   |   X     |  X   |     X    |
        +------------------+-------+---------+------+----------+
        | NDOF_MODE        |   X   |   X     |  X   |     X    |
        +------------------+-------+---------+------+----------+

        The default mode is ``NDOF_MODE``.
        """
        return self._read_register(_MODE_REGISTER)

    @property
    def calibration_status(self):
        """Tuple containing sys, gyro, accel, and mag calibration data."""
        calibration_data = self._read_register(_CALIBRATION_REGISTER)
        sys = (calibration_data >> 6) & 0x03
        gyro = (calibration_data >> 4) & 0x03
        accel = (calibration_data >> 2) & 0x03
        mag = calibration_data & 0x03
        return sys, gyro, accel, mag

    @property
    def calibrated(self):
        """Boolean indicating calibration status."""
        sys, gyro, accel, mag = self.calibration_status
        return sys == gyro == accel == mag == 0x03

    @mode.setter
    def mode(self, new_mode):
        self._write_register(_MODE_REGISTER, new_mode)

    @property
    def external_crystal(self):
        """Switches the use of external crystal on or off."""
        last_mode = self.mode
        self.mode = CONFIG_MODE
        self._write_register(_PAGE_REGISTER, 0x00)
        value = self._read_register(_TRIGGER_REGISTER)
        self.mode = last_mode
        return value == 0x80

    @external_crystal.setter
    def use_external_crystal(self, value):
        last_mode = self.mode
        self.mode = CONFIG_MODE
        self._write_register(_PAGE_REGISTER, 0x00)
        self._write_register(_TRIGGER_REGISTER, 0x80 if value else 0x00)
        self.mode = last_mode
        time.sleep(0.01)
