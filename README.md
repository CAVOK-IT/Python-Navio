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
$ wget https://github.com/bennierex/Python-Navio/archive/master.zip
$ unzip ./master.zip
$ cd ./Python-Navio-master
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

### Documentation
See the [wiki](https://github.com/bennierex/Python-Navio/wiki) for detailed documentation.
