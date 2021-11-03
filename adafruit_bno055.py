# SPDX-FileCopyrightText: 2017 Radomir Dopieralski for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_bno055`
=======================================================================================

This is a CircuitPython driver for the Bosch BNO055 nine degree of freedom
inertial measurement unit module with sensor fusion.

* Author(s): Radomir Dopieralski


**Hardware:**

* Adafruit `9-DOF Absolute Orientation IMU Fusion Breakout - BNO055
  <https://www.adafruit.com/product/4646>`_ (Product ID: 4646)


**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice

* Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register

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

ACCEL_2G = const(0x00)  # For accel_range property
ACCEL_4G = const(0x01)  # Default
ACCEL_8G = const(0x02)
ACCEL_16G = const(0x03)
ACCEL_7_81HZ = const(0x00)  # For accel_bandwidth property
ACCEL_15_63HZ = const(0x04)
ACCEL_31_25HZ = const(0x08)
ACCEL_62_5HZ = const(0x0C)  # Default
ACCEL_125HZ = const(0x10)
ACCEL_250HZ = const(0x14)
ACCEL_500HZ = const(0x18)
ACCEL_1000HZ = const(0x1C)
ACCEL_NORMAL_MODE = const(0x00)  # Default. For accel_mode property
ACCEL_SUSPEND_MODE = const(0x20)
ACCEL_LOWPOWER1_MODE = const(0x40)
ACCEL_STANDBY_MODE = const(0x60)
ACCEL_LOWPOWER2_MODE = const(0x80)
ACCEL_DEEPSUSPEND_MODE = const(0xA0)

GYRO_2000_DPS = const(0x00)  # Default. For gyro_range property
GYRO_1000_DPS = const(0x01)
GYRO_500_DPS = const(0x02)
GYRO_250_DPS = const(0x03)
GYRO_125_DPS = const(0x04)
GYRO_523HZ = const(0x00)  # For gyro_bandwidth property
GYRO_230HZ = const(0x08)
GYRO_116HZ = const(0x10)
GYRO_47HZ = const(0x18)
GYRO_23HZ = const(0x20)
GYRO_12HZ = const(0x28)
GYRO_64HZ = const(0x30)
GYRO_32HZ = const(0x38)  # Default
GYRO_NORMAL_MODE = const(0x00)  # Default. For gyro_mode property
GYRO_FASTPOWERUP_MODE = const(0x01)
GYRO_DEEPSUSPEND_MODE = const(0x02)
GYRO_SUSPEND_MODE = const(0x03)
GYRO_ADVANCEDPOWERSAVE_MODE = const(0x04)

MAGNET_2HZ = const(0x00)  # For magnet_rate property
MAGNET_6HZ = const(0x01)
MAGNET_8HZ = const(0x02)
MAGNET_10HZ = const(0x03)
MAGNET_15HZ = const(0x04)
MAGNET_20HZ = const(0x05)  # Default
MAGNET_25HZ = const(0x06)
MAGNET_30HZ = const(0x07)
MAGNET_LOWPOWER_MODE = const(0x00)  # For magnet_operation_mode property
MAGNET_REGULAR_MODE = const(0x08)  # Default
MAGNET_ENHANCEDREGULAR_MODE = const(0x10)
MAGNET_ACCURACY_MODE = const(0x18)
MAGNET_NORMAL_MODE = const(0x00)  # for magnet_power_mode property
MAGNET_SLEEP_MODE = const(0x20)
MAGNET_SUSPEND_MODE = const(0x40)
MAGNET_FORCEMODE_MODE = const(0x60)  # Default

_POWER_NORMAL = const(0x00)
_POWER_LOW = const(0x01)
_POWER_SUSPEND = const(0x02)

_MODE_REGISTER = const(0x3D)
_PAGE_REGISTER = const(0x07)
_ACCEL_CONFIG_REGISTER = const(0x08)
_MAGNET_CONFIG_REGISTER = const(0x09)
_GYRO_CONFIG_0_REGISTER = const(0x0A)
_GYRO_CONFIG_1_REGISTER = const(0x0B)
_CALIBRATION_REGISTER = const(0x35)
_OFFSET_ACCEL_REGISTER = const(0x55)
_OFFSET_MAGNET_REGISTER = const(0x5B)
_OFFSET_GYRO_REGISTER = const(0x61)
_RADIUS_ACCEL_REGISTER = const(0x67)
_RADIUS_MAGNET_REGISTER = const(0x69)
_TRIGGER_REGISTER = const(0x3F)
_POWER_REGISTER = const(0x3E)
_ID_REGISTER = const(0x00)
# Axis remap registers and values
_AXIS_MAP_CONFIG_REGISTER = const(0x41)
_AXIS_MAP_SIGN_REGISTER = const(0x42)
AXIS_REMAP_X = const(0x00)
AXIS_REMAP_Y = const(0x01)
AXIS_REMAP_Z = const(0x02)
AXIS_REMAP_POSITIVE = const(0x00)
AXIS_REMAP_NEGATIVE = const(0x01)


class _ScaledReadOnlyStruct(Struct):  # pylint: disable=too-few-public-methods
    def __init__(self, register_address, struct_format, scale):
        super().__init__(register_address, struct_format)
        self.scale = scale

    def __get__(self, obj, objtype=None):
        result = super().__get__(obj, objtype)
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


class BNO055:  # pylint: disable=too-many-public-methods
    """
    Base class for the BNO055 9DOF IMU sensor.

    **Quickstart: Importing and using the device**

        Here is an example of using the :class:`BNO055` class.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import board
            import adafruit_bno055

        Once this is done you can define your `board.I2C` object and define your sensor object

        .. code-block:: python

            i2c = board.I2C()  # uses board.SCL and board.SDA
            sensor = adafruit_bno055.BNO055_I2C(i2c)


        Now you have access to the :attr:`acceleration` attribute among others

        .. code-block:: python

            sensor = accelerometer.acceleration

    """

    def __init__(self):
        chip_id = self._read_register(_ID_REGISTER)
        if chip_id != _CHIP_ID:
            raise RuntimeError("bad chip id (%x != %x)" % (chip_id, _CHIP_ID))
        self._reset()
        self._write_register(_POWER_REGISTER, _POWER_NORMAL)
        self._write_register(_PAGE_REGISTER, 0x00)
        self._write_register(_TRIGGER_REGISTER, 0x00)
        self.accel_range = ACCEL_4G
        self.gyro_range = GYRO_2000_DPS
        self.magnet_rate = MAGNET_20HZ
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
        legend: x=on, -=off (see Table 3-3 in datasheet)

        +------------------+-------+---------+------+----------+----------+
        | Mode             | Accel | Compass | Gyro | Fusion   | Fusion   |
        |                  |       | (Mag)   |      | Absolute | Relative |
        +==================+=======+=========+======+==========+==========+
        | CONFIG_MODE      |   -   |   -     |  -   |     -    |     -    |
        +------------------+-------+---------+------+----------+----------+
        | ACCONLY_MODE     |   X   |   -     |  -   |     -    |     -    |
        +------------------+-------+---------+------+----------+----------+
        | MAGONLY_MODE     |   -   |   X     |  -   |     -    |     -    |
        +------------------+-------+---------+------+----------+----------+
        | GYRONLY_MODE     |   -   |   -     |  X   |     -    |     -    |
        +------------------+-------+---------+------+----------+----------+
        | ACCMAG_MODE      |   X   |   X     |  -   |     -    |     -    |
        +------------------+-------+---------+------+----------+----------+
        | ACCGYRO_MODE     |   X   |   -     |  X   |     -    |     -    |
        +------------------+-------+---------+------+----------+----------+
        | MAGGYRO_MODE     |   -   |   X     |  X   |     -    |     -    |
        +------------------+-------+---------+------+----------+----------+
        | AMG_MODE         |   X   |   X     |  X   |     -    |     -    |
        +------------------+-------+---------+------+----------+----------+
        | IMUPLUS_MODE     |   X   |   -     |  X   |     -    |     X    |
        +------------------+-------+---------+------+----------+----------+
        | COMPASS_MODE     |   X   |   X     |  -   |     X    |     -    |
        +------------------+-------+---------+------+----------+----------+
        | M4G_MODE         |   X   |   X     |  -   |     -    |     X    |
        +------------------+-------+---------+------+----------+----------+
        | NDOF_FMC_OFF_MODE|   X   |   X     |  X   |     X    |     -    |
        +------------------+-------+---------+------+----------+----------+
        | NDOF_MODE        |   X   |   X     |  X   |     X    |     -    |
        +------------------+-------+---------+------+----------+----------+

        The default mode is :const:`NDOF_MODE`.

        | You can set the mode using the line below:
        | ``sensor.mode = adafruit_bno055.ACCONLY_MODE``
        | replacing :const:`ACCONLY_MODE` with the mode you want to use

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
        return self._read_register(_MODE_REGISTER) & 0b00001111  # Datasheet Table 4-2

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
        if self.mode not in [0x00, 0x01, 0x03, 0x05, 0x08]:
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
        if self.mode in [0x08, 0x09, 0x0A, 0x0B, 0x0C]:
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
        if self.mode in [0x08, 0x09, 0x0A, 0x0B, 0x0C]:
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
        if self.mode in [0x08, 0x09, 0x0A, 0x0B, 0x0C]:
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
        if self.mode in [0x08, 0x09, 0x0A, 0x0B, 0x0C]:
            return self._gravity
        return (None, None, None)

    @property
    def _gravity(self):
        raise NotImplementedError("Must be implemented.")

    @property
    def accel_range(self):
        """Switch the accelerometer range and return the new range. Default value: +/- 4g
        See table 3-8 in the datasheet.
        """
        self._write_register(_PAGE_REGISTER, 0x01)
        value = self._read_register(_ACCEL_CONFIG_REGISTER)
        self._write_register(_PAGE_REGISTER, 0x00)
        return 0b00000011 & value

    @accel_range.setter
    def accel_range(self, rng=ACCEL_4G):
        self._write_register(_PAGE_REGISTER, 0x01)
        value = self._read_register(_ACCEL_CONFIG_REGISTER)
        masked_value = 0b11111100 & value
        self._write_register(_ACCEL_CONFIG_REGISTER, masked_value | rng)
        self._write_register(_PAGE_REGISTER, 0x00)

    @property
    def accel_bandwidth(self):
        """Switch the accelerometer bandwidth and return the new bandwidth. Default value: 62.5 Hz
        See table 3-8 in the datasheet.
        """
        self._write_register(_PAGE_REGISTER, 0x01)
        value = self._read_register(_ACCEL_CONFIG_REGISTER)
        self._write_register(_PAGE_REGISTER, 0x00)
        return 0b00011100 & value

    @accel_bandwidth.setter
    def accel_bandwidth(self, bandwidth=ACCEL_62_5HZ):
        if self.mode in [0x08, 0x09, 0x0A, 0x0B, 0x0C]:
            raise RuntimeError("Mode must not be a fusion mode")
        self._write_register(_PAGE_REGISTER, 0x01)
        value = self._read_register(_ACCEL_CONFIG_REGISTER)
        masked_value = 0b11100011 & value
        self._write_register(_ACCEL_CONFIG_REGISTER, masked_value | bandwidth)
        self._write_register(_PAGE_REGISTER, 0x00)

    @property
    def accel_mode(self):
        """Switch the accelerometer mode and return the new mode. Default value: Normal
        See table 3-8 in the datasheet.
        """
        self._write_register(_PAGE_REGISTER, 0x01)
        value = self._read_register(_ACCEL_CONFIG_REGISTER)
        self._write_register(_PAGE_REGISTER, 0x00)
        return 0b11100000 & value

    @accel_mode.setter
    def accel_mode(self, mode=ACCEL_NORMAL_MODE):
        if self.mode in [0x08, 0x09, 0x0A, 0x0B, 0x0C]:
            raise RuntimeError("Mode must not be a fusion mode")
        self._write_register(_PAGE_REGISTER, 0x01)
        value = self._read_register(_ACCEL_CONFIG_REGISTER)
        masked_value = 0b00011111 & value
        self._write_register(_ACCEL_CONFIG_REGISTER, masked_value | mode)
        self._write_register(_PAGE_REGISTER, 0x00)

    @property
    def gyro_range(self):
        """Switch the gyroscope range and return the new range. Default value: 2000 dps
        See table 3-9 in the datasheet.
        """
        self._write_register(_PAGE_REGISTER, 0x01)
        value = self._read_register(_GYRO_CONFIG_0_REGISTER)
        self._write_register(_PAGE_REGISTER, 0x00)
        return 0b00000111 & value

    @gyro_range.setter
    def gyro_range(self, rng=GYRO_2000_DPS):
        if self.mode in [0x08, 0x09, 0x0A, 0x0B, 0x0C]:
            raise RuntimeError("Mode must not be a fusion mode")
        self._write_register(_PAGE_REGISTER, 0x01)
        value = self._read_register(_GYRO_CONFIG_0_REGISTER)
        masked_value = 0b00111000 & value
        self._write_register(_GYRO_CONFIG_0_REGISTER, masked_value | rng)
        self._write_register(_PAGE_REGISTER, 0x00)

    @property
    def gyro_bandwidth(self):
        """Switch the gyroscope bandwidth and return the new bandwidth. Default value: 32 Hz
        See table 3-9 in the datasheet.
        """
        self._write_register(_PAGE_REGISTER, 0x01)
        value = self._read_register(_GYRO_CONFIG_0_REGISTER)
        self._write_register(_PAGE_REGISTER, 0x00)
        return 0b00111000 & value

    @gyro_bandwidth.setter
    def gyro_bandwidth(self, bandwidth=GYRO_32HZ):
        if self.mode in [0x08, 0x09, 0x0A, 0x0B, 0x0C]:
            raise RuntimeError("Mode must not be a fusion mode")
        self._write_register(_PAGE_REGISTER, 0x01)
        value = self._read_register(_GYRO_CONFIG_0_REGISTER)
        masked_value = 0b00000111 & value
        self._write_register(_GYRO_CONFIG_0_REGISTER, masked_value | bandwidth)
        self._write_register(_PAGE_REGISTER, 0x00)

    @property
    def gyro_mode(self):
        """Switch the gyroscope mode and return the new mode. Default value: Normal
        See table 3-9 in the datasheet.
        """
        self._write_register(_PAGE_REGISTER, 0x01)
        value = self._read_register(_GYRO_CONFIG_1_REGISTER)
        self._write_register(_PAGE_REGISTER, 0x00)
        return 0b00000111 & value

    @gyro_mode.setter
    def gyro_mode(self, mode=GYRO_NORMAL_MODE):
        if self.mode in [0x08, 0x09, 0x0A, 0x0B, 0x0C]:
            raise RuntimeError("Mode must not be a fusion mode")
        self._write_register(_PAGE_REGISTER, 0x01)
        value = self._read_register(_GYRO_CONFIG_1_REGISTER)
        masked_value = 0b00000000 & value
        self._write_register(_GYRO_CONFIG_1_REGISTER, masked_value | mode)
        self._write_register(_PAGE_REGISTER, 0x00)

    @property
    def magnet_rate(self):
        """Switch the magnetometer data output rate and return the new rate. Default value: 20Hz
        See table 3-10 in the datasheet.
        """
        self._write_register(_PAGE_REGISTER, 0x01)
        value = self._read_register(_MAGNET_CONFIG_REGISTER)
        self._write_register(_PAGE_REGISTER, 0x00)
        return 0b00000111 & value

    @magnet_rate.setter
    def magnet_rate(self, rate=MAGNET_20HZ):
        if self.mode in [0x08, 0x09, 0x0A, 0x0B, 0x0C]:
            raise RuntimeError("Mode must not be a fusion mode")
        self._write_register(_PAGE_REGISTER, 0x01)
        value = self._read_register(_MAGNET_CONFIG_REGISTER)
        masked_value = 0b01111000 & value
        self._write_register(_MAGNET_CONFIG_REGISTER, masked_value | rate)
        self._write_register(_PAGE_REGISTER, 0x00)

    @property
    def magnet_operation_mode(self):
        """Switch the magnetometer operation mode and return the new mode. Default value: Regular
        See table 3-10 in the datasheet.
        """
        self._write_register(_PAGE_REGISTER, 0x01)
        value = self._read_register(_MAGNET_CONFIG_REGISTER)
        self._write_register(_PAGE_REGISTER, 0x00)
        return 0b00011000 & value

    @magnet_operation_mode.setter
    def magnet_operation_mode(self, mode=MAGNET_REGULAR_MODE):
        if self.mode in [0x08, 0x09, 0x0A, 0x0B, 0x0C]:
            raise RuntimeError("Mode must not be a fusion mode")
        self._write_register(_PAGE_REGISTER, 0x01)
        value = self._read_register(_MAGNET_CONFIG_REGISTER)
        masked_value = 0b01100111 & value
        self._write_register(_MAGNET_CONFIG_REGISTER, masked_value | mode)
        self._write_register(_PAGE_REGISTER, 0x00)

    @property
    def magnet_mode(self):
        """Switch the magnetometer power mode and return the new mode. Default value: Forced
        See table 3-10 in the datasheet.
        """
        self._write_register(_PAGE_REGISTER, 0x01)
        value = self._read_register(_MAGNET_CONFIG_REGISTER)
        self._write_register(_PAGE_REGISTER, 0x00)
        return 0b01100000 & value

    @magnet_mode.setter
    def magnet_mode(self, mode=MAGNET_FORCEMODE_MODE):
        if self.mode in [0x08, 0x09, 0x0A, 0x0B, 0x0C]:
            raise RuntimeError("Mode must not be a fusion mode")
        self._write_register(_PAGE_REGISTER, 0x01)
        value = self._read_register(_MAGNET_CONFIG_REGISTER)
        masked_value = 0b00011111 & value
        self._write_register(_MAGNET_CONFIG_REGISTER, masked_value | mode)
        self._write_register(_PAGE_REGISTER, 0x00)

    def _write_register(self, register, value):
        raise NotImplementedError("Must be implemented.")

    def _read_register(self, register):
        raise NotImplementedError("Must be implemented.")

    @property
    def axis_remap(self):
        """Return a tuple with the axis remap register values.

        This will return 6 values with the following meaning:
          - X axis remap (a value of AXIS_REMAP_X, AXIS_REMAP_Y, or AXIS_REMAP_Z.
                          which indicates that the physical X axis of the chip
                          is remapped to a different axis)
          - Y axis remap (see above)
          - Z axis remap (see above)
          - X axis sign (a value of AXIS_REMAP_POSITIVE or AXIS_REMAP_NEGATIVE
                         which indicates if the X axis values should be positive/
                         normal or negative/inverted.  The default is positive.)
          - Y axis sign (see above)
          - Z axis sign (see above)

        Note that the default value, per the datasheet, is NOT P0,
        but rather P1 ()
        """
        # Get the axis remap register value.
        map_config = self._read_register(_AXIS_MAP_CONFIG_REGISTER)
        z = (map_config >> 4) & 0x03
        y = (map_config >> 2) & 0x03
        x = map_config & 0x03
        # Get the axis remap sign register value.
        sign_config = self._read_register(_AXIS_MAP_SIGN_REGISTER)
        x_sign = (sign_config >> 2) & 0x01
        y_sign = (sign_config >> 1) & 0x01
        z_sign = sign_config & 0x01
        # Return the results as a tuple of all 3 values.
        return (x, y, z, x_sign, y_sign, z_sign)

    @axis_remap.setter
    def axis_remap(self, remap):
        """Pass a tuple consisting of x, y, z, x_sign, y-sign, and z_sign.

        Set axis remap for each axis.  The x, y, z parameter values should
        be set to one of AXIS_REMAP_X (0x00), AXIS_REMAP_Y (0x01), or
        AXIS_REMAP_Z (0x02) and will change the BNO's axis to represent another
        axis.  Note that two axises cannot be mapped to the same axis, so the
        x, y, z params should be a unique combination of AXIS_REMAP_X,
        AXIS_REMAP_Y, AXIS_REMAP_Z values.
        The x_sign, y_sign, z_sign values represent if the axis should be
        positive or negative (inverted). See section 3.4 of the datasheet for
        information on the proper settings for each possible orientation of
        the chip.
        """
        x, y, z, x_sign, y_sign, z_sign = remap
        # Switch to configuration mode. Necessary to remap axes
        current_mode = self._read_register(_MODE_REGISTER)
        self.mode = CONFIG_MODE
        # Set the axis remap register value.
        map_config = 0x00
        map_config |= (z & 0x03) << 4
        map_config |= (y & 0x03) << 2
        map_config |= x & 0x03
        self._write_register(_AXIS_MAP_CONFIG_REGISTER, map_config)
        # Set the axis remap sign register value.
        sign_config = 0x00
        sign_config |= (x_sign & 0x01) << 2
        sign_config |= (y_sign & 0x01) << 1
        sign_config |= z_sign & 0x01
        self._write_register(_AXIS_MAP_SIGN_REGISTER, sign_config)
        # Go back to normal operation mode.
        self._write_register(_MODE_REGISTER, current_mode)


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

    def _write_register(
        self, register, data
    ):  # pylint: disable=arguments-differ,arguments-renamed
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
        return tuple(x / 100 for x in resp)

    @property
    def _magnetic(self):
        resp = struct.unpack("<hhh", self._read_register(0x0E, 6))
        return tuple(x / 16 for x in resp)

    @property
    def _gyro(self):
        resp = struct.unpack("<hhh", self._read_register(0x14, 6))
        return tuple(x * 0.001090830782496456 for x in resp)

    @property
    def _euler(self):
        resp = struct.unpack("<hhh", self._read_register(0x1A, 6))
        return tuple(x / 16 for x in resp)

    @property
    def _quaternion(self):
        resp = struct.unpack("<hhhh", self._read_register(0x20, 8))
        return tuple(x / (1 << 14) for x in resp)

    @property
    def _linear_acceleration(self):
        resp = struct.unpack("<hhh", self._read_register(0x28, 6))
        return tuple(x / 100 for x in resp)

    @property
    def _gravity(self):
        resp = struct.unpack("<hhh", self._read_register(0x2E, 6))
        return tuple(x / 100 for x in resp)

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
