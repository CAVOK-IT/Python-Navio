Python-Navio
============

Python extension for the [Navio shield](http://www.emlid.com/) for [Raspberry Pi](http://www.raspberrypi.org/).

## Important!!! ##
This extension is in pre-alpha state. It is in no way stable and guaranteed to work correctly. Please use at your own risk!
There are several (mayor) bugs still to fix and functionality to add, so please check back often for revised code and examples.

### Installing ###
```bash
$ sudo apt-get update
$ sudo apt-get install python-dev
$ python setup.py install
```

### Basic usage ###
```python
>>> import navio
>>> nio = navio.Navio()

>>> nio.IMU_get_motion9()
((0.74267578125, 0.60009765625, -0.2841796875), (5.060975551605225, 0.6097561120986938, -1.8902438879013062), (0.0, 0.0, 0.0))

>>> nio.PWM_set_led(255,0,0)    # set LED to full brightness red

>>> nio.BARO_get_temp_and_press()
{'press': 999.8275756835938, 'temp': 34.181156158447266}

>>> nio.FRAM_write(0, b'abcdefg')
1
>>> nio.FRAM_read(0, 7)
b'abcdefg'

>>> from navio import UBX_MSG_NAV_POSLLH
>>> import struct
>>> from collections import namedtuple
>>>
>>> msg_fields = nio.GPS_get_messagefields(UBX_MSG_NAV_POSLLH)
>>> msg_format = nio.GPS_get_messageformat(UBX_MSG_NAV_POSLLH)
>>> PosLLH = namedtuple('PosLLH', msg_fields)
>>> msg_data = nio.GPS_get_message(UBX_MSG_NAV_POSLLH)
>>> message = PosLLH._make(struct.unpack(msg_format, msg_data))
>>> print(message)
PosLLH(Sync1=181, Sync2=98, clsID=1, msgID=2, payloadLength=28, iTOW=309742000, lon=0, lat=0, height=0, hMSL=-17000, hAcc=4294967295, vAcc=3750030848, CK_A=64, CK_B=26)
```


### Detailed usage ###
#### Setup and initialization ####
```python
>>> import navio
>>> from navio import RPI_MODEL_A, BANANA_PI, BIT_COMPONENTS_GPS, BIT_COMPONENTS_IMU    # import what you need. Check navio.__dict__ for all available options.
>>> 
>>> nio = navio.Navio()               # for RPi models B and B+ (default)
>>> # OR
>>> nio = navio.Navio(RPI_MODEL_A)    # for RPi model A
>>> # OR
>>> nio = navio.Navio(BANANA_PI)      # for Banana Pi
>>> # OR
>>> nio = navio.Navio(rpi_model=RPI_MODEL_A, enabled_components=BIT_COMPONENTS_GPS|BIT_COMPONENTS_IMU) # model A, only enable GPS and IMU
>>> # etc.
```

#### IMU ####
IMU Motion6 and Motion9 outputs.
```python
>>> # Read accelerometer and gyroscope data
>>> ((ax, ay, az), (gx, gy, gz)) = nio.IMU_get_motion6()
>>> # Read accelerometer, gyroscope and magnetometer data
>>> ((ax, ay, az), (gx, gy, gz), (mx, my, mz)) = nio.IMU_get_motion9()
```

#### PWM and LED ####
Set the RGB led.
```python
>>> # Set LED RGB value
>>> nio.PWM_set_led(255, 0, 0)      # red
>>> nio.PWM_set_led(0, 255, 0)      # green
>>> nio.PWM_set_led(0, 0, 255)      # blue
>>> nio.PWM_set_led(125, 125, 125)  # approx half-intensity white
>>> nio.PWM_set_led(0, 0, 0)        # off
```
Set PWM output for a specific channel. The available channels are 1 - 13, the available values are 0 - 4095.
```python
>>> # Set PWM output
>>> # Available channels is 1 - 13
>>> # Available values are 0 - 4095
>>> nio.PWM_set(4, 2048)    # set channel 4 to a value of 2048
```

#### ADC ####
Read mV from ADC:
```python
>>> nio.ADC_get_millivolts()
571.75
```

#### Baro ####
Get a temperature and pressure reading.
```python
>>> nio.BARO_get_temp_and_press()
{'press': 999.9314575195312, 'temp': 34.317222595214844}
```

#### FRAM ####
Read from and write data to FRAM.
```python
>>> nio.FRAM_write(0, b'abcdefg')
1
>>> nio.FRAM_read(0, 7)
b'abcdefg'
```
Perform FRAM test:
```python
>>> # Perform FRAM read/write test
>>> "FRAM test OK!" if nio.FRAM_test() else "FRAM test failed!"
'FRAM test OK!'
```
