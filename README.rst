Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-bno055/badge/?version=latest
    :target: https://docs.circuitpython.org/projects/bno055/en/latest/
    :alt: Documentation Status

.. image :: https://img.shields.io/discord/327254708534116352.svg
    :target: https://adafru.it/discord
    :alt: Discord

.. image:: https://github.com/adafruit/Adafruit_CircuitPython_BNO055/workflows/Build%20CI/badge.svg
    :target: https://github.com/adafruit/Adafruit_CircuitPython_BNO055/actions/
    :alt: Build Status


Dependencies
=============

This driver depends on the `Register
<https://github.com/adafruit/Adafruit_CircuitPython_Register>`_ and `Bus Device
<https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_ libraries.
Please ensure they are also available on the CircuitPython filesystem.  This is
easily achieved by downloading `a library and driver bundle
<https://github.com/adafruit/Adafruit_CircuitPython_Bundle>`_.

Installing from PyPI
====================

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from
PyPI <https://pypi.org/project/adafruit-circuitpython-bno055/>`_. To install for current user:

.. code-block:: shell

    pip3 install adafruit-circuitpython-bno055

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install adafruit-circuitpython-bno055

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .env
    source .env/bin/activate
    pip3 install adafruit-circuitpython-bno055

Usage Notes
===========

Of course, you must import the library to use it:

.. code:: python3

    import adafruit_bno055


This driver takes an instantiated and active I2C object as an argument to its
constructor.  The way to create an I2C object depends on the board you are
using. For boards with labeled SCL and SDA pins, you can:

.. code:: python3

    import board

    i2c = board.I2C()

Once you have the I2C object, you can create the sensor object:

.. code:: python3

    sensor = adafruit_bno055.BNO055_I2C(i2c)


And then you can start reading the measurements:

.. code:: python3

    print(sensor.temperature)
    print(sensor.euler)
    print(sensor.gravity)


Documentation
=============

API documentation for this library can be found on `Read the Docs <https://docs.circuitpython.org/projects/bno055/en/latest/>`_.

For information on building library documentation, please check out `this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/Adafruit_CircuitPython_bno055/blob/main/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.
