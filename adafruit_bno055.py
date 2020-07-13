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
import struct

from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_struct import Struct, UnaryStruct

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_BNO055.git"

_CHIP_ID = const(0xA0)

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
M4G_MODE = const(0x0A)
NDOF_FMC_OFF_MODE = const(0x0B)
NDOF_MODE = const(0x0C)

_POWER_NORMAL = const(0x00)
_POWER_LOW = const(0x01)
_POWER_SUSPEND = const(0x02)

_MODE_REGISTER = const(0x3D)
_PAGE_REGISTER = const(0x07)
_CALIBRATION_REGISTER = const(0x35)
_OFFSET_ACCEL_REGISTER = const(0x55)
_OFFSET_MAGNET_REGISTER = const(0x5B)
_OFFSET_GYRO_REGISTER = const(0x61)
_RADIUS_ACCEL_REGISTER = const(0x67)
_RADIUS_MAGNET_REGISTER = const(0x69)
_TRIGGER_REGISTER = const(0x3F)
_POWER_REGISTER = const(0x3E)
_ID_REGISTER = const(0x00)


class _ScaledReadOnlyStruct(Struct):  # pylint: disable=too-few-public-methods
    def __init__(self, register_address, struct_format, scale):
        super(_ScaledReadOnlyStruct, self).__init__(register_address, struct_format)
        self.scale = scale

    def __get__(self, obj, objtype=None):
        result = super(_ScaledReadOnlyStruct, self).__get__(obj, objtype)
        return tuple(self.scale * v for v in result)

    def __set__(self, obj, value):
        raise NotImplementedError()


class _ReadOnlyUnaryStruct(UnaryStruct):  # pylint: disable=too-few-public-methods
    def __set__(self, obj, value):
        raise NotImplementedError()


class _ModeStruct(Struct):  # pylint: disable=too-few-public-methods
    def __init__(self, register_address, struct_format, mode):
        super().__init__(register_address, struct_format)
        self.mode = mode

    def __get__(self, obj, objtype=None):
        last_mode = obj.mode
        obj.mode = self.mode
        result = super().__get__(obj, objtype)
        obj.mode = last_mode
        # single value comes back as a one-element tuple
        return result[0] if isinstance(result, tuple) and len(result) == 1 else result

    def __set__(self, obj, value):
        last_mode = obj.mode
        obj.mode = self.mode
        # underlying __set__() expects a tuple
        set_val = value if isinstance(value, tuple) else (value,)
        super().__set__(obj, set_val)
        obj.mode = last_mode


class BNO055:
    """
    Base class for the BNO055 9DOF IMU sensor.
    """

    def __init__(self):
        chip_id = self._read_register(_ID_REGISTER)
        if chip_id != _CHIP_ID:
            raise RuntimeError("bad chip id (%x != %x)" % (chip_id, _CHIP_ID))
        self._reset()
        self._write_register(_POWER_REGISTER, _POWER_NORMAL)
        self._write_register(_PAGE_REGISTER, 0x00)
        self._write_register(_TRIGGER_REGISTER, 0x00)
        time.sleep(0.01)
        self.mode = NDOF_MODE
        time.sleep(0.01)

    def _reset(self):
        """Resets the sensor to default settings."""
        self.mode = CONFIG_MODE
        try:
            self._write_register(_TRIGGER_REGISTER, 0x20)
        except OSError:  # error due to the chip resetting
            pass
        # wait for the chip to reset (650 ms typ.)
        time.sleep(0.7)

    @property
    def mode(self):
        """
        legend: x=on, -=off

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

        | You can set the mode using the line below:
        | ``sensor.mode = adafruit_bno055.ACCONLY_MODE``
        | replacing ``ACCONLY_MODE`` with the mode you want to use

        .. data:: CONFIG_MODE

           This mode is used to configure BNO, wherein all output data is reset to zero and sensor
           fusion is halted.

        .. data:: ACCONLY_MODE

           In this mode, the BNO055 behaves like a stand-alone acceleration sensor. In this mode the
           other sensors (magnetometer, gyro) are suspended to lower the power consumption.

        .. data:: MAGONLY_MODE

           In MAGONLY mode, the BNO055 behaves like a stand-alone magnetometer, with acceleration
           sensor and gyroscope being suspended.

        .. data:: GYRONLY_MODE

           In GYROONLY mode, the BNO055 behaves like a stand-alone gyroscope, with acceleration
           sensor and magnetometer being suspended.

        .. data:: ACCMAG_MODE

           Both accelerometer and magnetometer are switched on, the user can read the data from
           these two sensors.

        .. data:: ACCGYRO_MODE

           Both accelerometer and gyroscope are switched on; the user can read the data from these
           two sensors.

        .. data:: MAGGYRO_MODE

           Both magnetometer and gyroscope are switched on, the user can read the data from these
           two sensors.

        .. data:: AMG_MODE

           All three sensors accelerometer, magnetometer and gyroscope are switched on.

        .. data:: IMUPLUS_MODE

           In the IMU mode the relative orientation of the BNO055 in space is calculated from the
           accelerometer and gyroscope data. The calculation is fast (i.e. high output data rate).

        .. data:: COMPASS_MODE

           The COMPASS mode is intended to measure the magnetic earth field and calculate the
           geographic direction.

        .. data:: M4G_MODE

           The M4G mode is similar to the IMU mode, but instead of using the gyroscope signal to
           detect rotation, the changing orientation of the magnetometer in the magnetic field is
           used.

        .. data:: NDOF_FMC_OFF_MODE

           This fusion mode is same as NDOF mode, but with the Fast Magnetometer Calibration turned
           ‘OFF’.

        .. data:: NDOF_MODE

           This is a fusion mode with 9 degrees of freedom where the fused absolute orientation data
           is calculated from accelerometer, gyroscope and the magnetometer.

        """
        return self._read_register(_MODE_REGISTER)

    @mode.setter
    def mode(self, new_mode):
        self._write_register(_MODE_REGISTER, CONFIG_MODE)  # Empirically necessary
        time.sleep(0.02)  # Datasheet table 3.6
        if new_mode != CONFIG_MODE:
            self._write_register(_MODE_REGISTER, new_mode)
            time.sleep(0.01)  # Table 3.6

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

    @property
    def temperature(self):
        """Measures the temperature of the chip in degrees Celsius."""
        return self._temperature

    @property
    def _temperature(self):
        raise NotImplementedError("Must be implemented.")

    @property
    def acceleration(self):
        """Gives the raw accelerometer readings, in m/s.
        Returns an empty tuple of length 3 when this property has been disabled by the current mode.
        """
        if self.mode not in [0x00, 0x02, 0x03, 0x06]:
            return self._acceleration
        return (None, None, None)

    @property
    def _acceleration(self):
        raise NotImplementedError("Must be implemented.")

    @property
    def magnetic(self):
        """Gives the raw magnetometer readings in microteslas.
        Returns an empty tuple of length 3 when this property has been disabled by the current mode.
        """
        if self.mode not in [0x00, 0x03, 0x05, 0x08]:
            return self._magnetic
        return (None, None, None)

    @property
    def _magnetic(self):
        raise NotImplementedError("Must be implemented.")

    @property
    def gyro(self):
        """Gives the raw gyroscope reading in radians per second.
        Returns an empty tuple of length 3 when this property has been disabled by the current mode.
        """
        if self.mode not in [0x00, 0x01, 0x02, 0x04, 0x09, 0x0A]:
            return self._gyro
        return (None, None, None)

    @property
    def _gyro(self):
        raise NotImplementedError("Must be implemented.")

    @property
    def euler(self):
        """Gives the calculated orientation angles, in degrees.
        Returns an empty tuple of length 3 when this property has been disabled by the current mode.
        """
        if self.mode in [0x09, 0x0B, 0x0C]:
            return self._euler
        return (None, None, None)

    @property
    def _euler(self):
        raise NotImplementedError("Must be implemented.")

    @property
    def quaternion(self):
        """Gives the calculated orientation as a quaternion.
        Returns an empty tuple of length 3 when this property has been disabled by the current mode.
        """
        if self.mode in [0x09, 0x0B, 0x0C]:
            return self._quaternion
        return (None, None, None, None)

    @property
    def _quaternion(self):
        raise NotImplementedError("Must be implemented.")

    @property
    def linear_acceleration(self):
        """Returns the linear acceleration, without gravity, in m/s.
        Returns an empty tuple of length 3 when this property has been disabled by the current mode.
        """
        if self.mode in [0x09, 0x0B, 0x0C]:
            return self._linear_acceleration
        return (None, None, None)

    @property
    def _linear_acceleration(self):
        raise NotImplementedError("Must be implemented.")

    @property
    def gravity(self):
        """Returns the gravity vector, without acceleration in m/s.
        Returns an empty tuple of length 3 when this property has been disabled by the current mode.
        """
        if self.mode in [0x09, 0x0B, 0x0C]:
            return self._gravity
        return (None, None, None)

    @property
    def _gravity(self):
        raise NotImplementedError("Must be implemented.")

    def _write_register(self, register, value):
        raise NotImplementedError("Must be implemented.")

    def _read_register(self, register):
        raise NotImplementedError("Must be implemented.")


class BNO055_I2C(BNO055):
    """
    Driver for the BNO055 9DOF IMU sensor via I2C.
    """

    _temperature = _ReadOnlyUnaryStruct(0x34, "b")
    _acceleration = _ScaledReadOnlyStruct(0x08, "<hhh", 1 / 100)
    _magnetic = _ScaledReadOnlyStruct(0x0E, "<hhh", 1 / 16)
    _gyro = _ScaledReadOnlyStruct(0x14, "<hhh", 0.001090830782496456)
    _euler = _ScaledReadOnlyStruct(0x1A, "<hhh", 1 / 16)
    _quaternion = _ScaledReadOnlyStruct(0x20, "<hhhh", 1 / (1 << 14))
    _linear_acceleration = _ScaledReadOnlyStruct(0x28, "<hhh", 1 / 100)
    _gravity = _ScaledReadOnlyStruct(0x2E, "<hhh", 1 / 100)

    offsets_accelerometer = _ModeStruct(_OFFSET_ACCEL_REGISTER, "<hhh", CONFIG_MODE)
    """Calibration offsets for the accelerometer"""
    offsets_magnetometer = _ModeStruct(_OFFSET_MAGNET_REGISTER, "<hhh", CONFIG_MODE)
    """Calibration offsets for the magnetometer"""
    offsets_gyroscope = _ModeStruct(_OFFSET_GYRO_REGISTER, "<hhh", CONFIG_MODE)
    """Calibration offsets for the gyroscope"""

    radius_accelerometer = _ModeStruct(_RADIUS_ACCEL_REGISTER, "<h", CONFIG_MODE)
    """Radius for accelerometer (cm?)"""
    radius_magnetometer = _ModeStruct(_RADIUS_MAGNET_REGISTER, "<h", CONFIG_MODE)
    """Radius for magnetometer (cm?)"""

    def __init__(self, i2c, address=0x28):
        self.buffer = bytearray(2)
        self.i2c_device = I2CDevice(i2c, address)
        super().__init__()

    def _write_register(self, register, value):
        self.buffer[0] = register
        self.buffer[1] = value
        with self.i2c_device as i2c:
            i2c.write(self.buffer)

    def _read_register(self, register):
        self.buffer[0] = register
        with self.i2c_device as i2c:
            i2c.write_then_readinto(self.buffer, self.buffer, out_end=1, in_start=1)
        return self.buffer[1]


class BNO055_UART(BNO055):
    """
    Driver for the BNO055 9DOF IMU sensor via UART.
    """

    def __init__(self, uart):
        self._uart = uart
        self._uart.baudrate = 115200
        super().__init__()

    def _write_register(self, register, data):  # pylint: disable=arguments-differ
        if not isinstance(data, bytes):
            data = bytes([data])
        self._uart.write(bytes([0xAA, 0x00, register, len(data)]) + data)
        now = time.monotonic()
        while self._uart.in_waiting < 2 and time.monotonic() - now < 0.25:
            pass
        resp = self._uart.read(self._uart.in_waiting)
        if len(resp) < 2:
            raise OSError("UART access error.")
        if resp[0] != 0xEE or resp[1] != 0x01:
            raise RuntimeError("UART write error: {}".format(resp[1]))

    def _read_register(self, register, length=1):  # pylint: disable=arguments-differ
        i = 0
        while i < 3:
            self._uart.write(bytes([0xAA, 0x01, register, length]))
            now = time.monotonic()
            while self._uart.in_waiting < length + 2 and time.monotonic() - now < 0.1:
                pass
            resp = self._uart.read(self._uart.in_waiting)
            if len(resp) >= 2 and resp[0] == 0xBB:
                break
            i += 1
        if len(resp) < 2:
            raise OSError("UART access error.")
        if resp[0] != 0xBB:
            raise RuntimeError("UART read error: {}".format(resp[1]))
        if length > 1:
            return resp[2:]
        return int(resp[2])

    @property
    def _temperature(self):
        return self._read_register(0x34)

    @property
    def _acceleration(self):
        resp = struct.unpack("<hhh", self._read_register(0x08, 6))
        return tuple([x / 100 for x in resp])

    @property
    def _magnetic(self):
        resp = struct.unpack("<hhh", self._read_register(0x0E, 6))
        return tuple([x / 16 for x in resp])

    @property
    def _gyro(self):
        resp = struct.unpack("<hhh", self._read_register(0x14, 6))
        return tuple([x * 0.001090830782496456 for x in resp])

    @property
    def _euler(self):
        resp = struct.unpack("<hhh", self._read_register(0x1A, 6))
        return tuple([x / 16 for x in resp])

    @property
    def _quaternion(self):
        resp = struct.unpack("<hhhh", self._read_register(0x20, 8))
        return tuple([x / (1 << 14) for x in resp])

    @property
    def _linear_acceleration(self):
        resp = struct.unpack("<hhh", self._read_register(0x28, 6))
        return tuple([x / 100 for x in resp])

    @property
    def _gravity(self):
        resp = struct.unpack("<hhh", self._read_register(0x2E, 6))
        return tuple([x / 100 for x in resp])

    @property
    def offsets_accelerometer(self):
        """Calibration offsets for the accelerometer"""
        return struct.unpack("<hhh", self._read_register(_OFFSET_ACCEL_REGISTER, 6))

    @offsets_accelerometer.setter
    def offsets_accelerometer(self, offsets):
        data = bytearray(6)
        struct.pack_into("<hhh", data, 0, *offsets)
        self._write_register(_OFFSET_ACCEL_REGISTER, bytes(data))

    @property
    def offsets_magnetometer(self):
        """Calibration offsets for the magnetometer"""
        return struct.unpack("<hhh", self._read_register(_OFFSET_MAGNET_REGISTER, 6))

    @offsets_magnetometer.setter
    def offsets_magnetometer(self, offsets):
        data = bytearray(6)
        struct.pack_into("<hhh", data, 0, *offsets)
        self._write_register(_OFFSET_MAGNET_REGISTER, bytes(data))

    @property
    def offsets_gyroscope(self):
        """Calibration offsets for the gyroscope"""
        return struct.unpack("<hhh", self._read_register(_OFFSET_GYRO_REGISTER, 6))

    @offsets_gyroscope.setter
    def offsets_gyroscope(self, offsets):
        data = bytearray(6)
        struct.pack_into("<hhh", data, 0, *offsets)
        self._write_register(_OFFSET_GYRO_REGISTER, bytes(data))

    @property
    def radius_accelerometer(self):
        """Radius for accelerometer (cm?)"""
        return struct.unpack("<h", self._read_register(_RADIUS_ACCEL_REGISTER, 2))[0]

    @radius_accelerometer.setter
    def radius_accelerometer(self, radius):
        data = bytearray(2)
        struct.pack_into("<h", data, 0, radius)
        self._write_register(_RADIUS_ACCEL_REGISTER, bytes(data))

    @property
    def radius_magnetometer(self):
        """Radius for magnetometer (cm?)"""
        return struct.unpack("<h", self._read_register(_RADIUS_MAGNET_REGISTER, 2))[0]

    @radius_magnetometer.setter
    def radius_magnetometer(self, radius):
        data = bytearray(2)
        struct.pack_into("<h", data, 0, radius)
        self._write_register(_RADIUS_MAGNET_REGISTER, bytes(data))
