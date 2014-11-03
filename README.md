Python-Navio
============

Python extension for the [Navio shield](http://www.emlid.com/) for [Raspberry Pi](http://www.raspberrypi.org/).

### Important!!! ###
This extension is in pre-alpha state. It is in no way stable and guaranteed to work correctly. Please use at your own risk!
There are several (mayor) bugs still to fix and functionality to add, so please check back often for revised code and examples.

#### Installing ####
```bash
$ sudo apt-get update
$ sudo apt-get install python-dev
$ python setup.py install
```

#### Usage ####
```python
import navio
from navio import RPI_MODEL_A, BANANA_PI, BIT_COMPONENTS_GPS, BIT_COMPONENTS_IMU    # import what you need. Check navio.__dict__ for all available options.

nio = navio.Navio()                     # for RPi models B and B+
# OR
nio = navio.Navio(RPI_MODEL_A)    # for RPi model A
# OR
nio = navio.Navio(BANANA_PI)      # for Banana Pi
# OR
nio = navio.Navio(rpi_model=RPI_MODEL_A, enabled_components=BIT_COMPONENTS_GPS|BIT_COMPONENTS_IMU) # model A, only enable GPS and IMU
# etc.

# Read accelerometer and gyroscope data
((ax, ay, az), (gx, gy, gz)) = nio.get_motion6()
# Read accelerometer, gyroscope and magnetometer data
((ax, ay, az), (gx, gy, gz), (mx, my, mz)) = nio.get_motion9()

# Set LED RGB value
nio.set_led(255, 0, 0)      # red
nio.set_led(0, 255, 0)      # green
nio.set_led(0, 0, 255)      # blue
nio.set_led(125, 125, 125)  # approx half-intensity white
nio.set_led(0, 0, 0)        # off

# Read ADC
millivolts = nio.ADC_get_millivolts()

# Read temperature and barometric pressure
tempandpress = nio.get_temp_and_pressure()
print("Temperature: %s\nPressure: %s\n" % (tempandpress['temp'], tempandpress['press']))

# Set PWM output
# Available channels is 1 - 13
# Available values are 0 - 4095
nio.set_pwm(4, 2048)    # set channel 4 to a value of 2048

# Write/read bytes to/from FRAM
nio.FRAM_write(0, b'abcdefg')
print(nio.FRAM_read(0, 7))  # should output b'abcdefg'

# Perform FRAM read/write test
result = "FRAM test OK!" if nio.FRAM_test() else "FRAM test failed!"
print(result)
```
