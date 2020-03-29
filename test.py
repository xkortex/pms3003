#!/bin/python
from pms3003 import PMSensor
import RPi.GPIO as GPIO
import dht11

# run to get particle measures in the console

# call a PMSensor class
# 0 for indoor sensing, 1 for outdoor
with PMSensor() as pm:

    # print avg'ed PM1, PM2.5, PM10 values
    data = pm.read_pm()
    print(data)

# wakeup pms3003 (if necessary) and print a single read
# pm.write_serial('BM\xe4\x00\x01\x01t', 45)
# pm.write_serial('BM\xe1\x00\x01\x01q', 15)
# print(pm.single_read())

# setup dht11
# GPIO.setwarnings(False)
# GPIO.setmode(GPIO.BCM)
# GPIO.cleanup()
