#!/bin/python
import os
import sys
import json
from pms3003 import PMSensor
import RPi.GPIO as GPIO
import dht11
import util

# run to get particle measures in the console

# call a PMSensor class
# 0 for indoor sensing, 1 for outdoor
with PMSensor() as pm:
    fname = os.path.join(os.path.expanduser('~/.local/share/pms3003'), util.pathsafe_timestamp() + '.json.txt')
    util.make_path(fname, True)
    # print avg'ed PM1, PM2.5, PM10 values
    # store any metadata we dump in hastily
    with open(fname, 'a') as fp:
        fp.write('[')
        json.dump(sys.argv[1:], fp)
        fp.write('\n')
    while True:
        try:
            with open(fname, 'a') as fp:
                data = pm.sm_read_active_once()
                ds = json.dumps(data)
                fp.write(',\n' + ds)
                print('{}| {: >7} {: >7} {: >7} {}'.format(data['time'], data['pm1'], data['pm2.5'], data['pm10'], data.get('status', '')))
        except KeyboardInterrupt:
            print('current state: {}'.format(pm.m.state))
            break

    with open(fname, 'a') as fp:
        fp.write('\n]')

    print('Wrote to: \n{}'.format(fname))


# wakeup pms3003 (if necessary) and print a single read
# pm.write_serial('BM\xe4\x00\x01\x01t', 45)
# pm.write_serial('BM\xe1\x00\x01\x01q', 15)
# print(pm.single_read())

# setup dht11
# GPIO.setwarnings(False)
# GPIO.setmode(GPIO.BCM)
# GPIO.cleanup()
