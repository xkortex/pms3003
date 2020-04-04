#!bin/python
# -*- coding: utf-8 -*-
import serial
import time
import datetime
import numpy as np
from vprint import vprint


PMColumns = ['time', 'pm1', 'pm2.5', 'pm10', 'status']

class Commands(object):
    wakeup  = b'BM\xe4\x00\x01\x01t'
    active  = b'BM\xe1\x00\x01\x01q'
    passive = b'BM\xe1\x00\x00\x01p'
    standby = b'BM\xe4\x00\x00\x01s'

def split_by_n(seq, n=2):
    '''A generator to divide a sequence into chunks of n units.'''
    while seq:
        yield seq[:n]
        seq = seq[n:]


def prettyhex(bytes: bytes, delim: str = ' '):
    return delim.join(split_by_n(bytes.hex()))


def fmt_packet(bytes: bytes, delim: str = ' '):
    return '{: >3} >{}'.format(len(bytes), prettyhex(bytes))


def parse_packet(packet: bytes, standard=0):
    data_hex = packet.hex()

    # calculate pm values
    try:
        # indoor
        if standard == 0:
            pm1 = int(data_hex[4] + data_hex[5] + data_hex[6] + data_hex[7], 16)
            pm25 = int(data_hex[8] + data_hex[9] + data_hex[10] + data_hex[11], 16)
            pm10 = int(data_hex[12] + data_hex[13] + data_hex[14] + data_hex[15], 16)
        # outdoor
        # todo: broken?
        elif standard == 1:
            pm1 = int(data_hex[16] + data_hex[17] + data_hex[18] + data_hex[19], 16)
            pm25 = int(data_hex[20] + data_hex[21] + data_hex[22] + data_hex[23], 16)
            pm10 = int(data_hex[24] + data_hex[25] + data_hex[26] + data_hex[27], 16)

        return [pm1, pm25, pm10, '']
    except IndexError:
        print('stupid thing broke. actual length: {} \n{}'.format(len(data_hex), fmt_packet(packet)))
        return [np.nan, np.nan, np.nan, 'failed to parse']

class PMSensor(object):

    baudrate = 9600 # hardcode the baudrate

    def __init__(self, outdoor=0, port='/dev/ttyS0', timeout=10):

        # hardcode the gpio serial port
        # /dev/ttyAMA0 -> Bluetooth (or GPIO when Bluetooth module turned off)
        # /dev/ttyS0 -> GPIO serial port
        self.port = port

        # sensor placed indoor/outdoor (0/1)
        self.standard = outdoor

        # enable software flow control.
        self.xonxoff = True

        self.timeout = timeout

        self.serial = None  # type: serial.Serial

    def __enter__(self):
        self.open_port()
        self.cmd_wakeup()
        return self

    def __exit__(self, type, value, traceback):
        self.cmd_standby()
        self.close()

    def open_port(self):
        vprint('Opening port {} @ {} ...'.format(self.port, self.baudrate))

        # open serial port
        self.serial = serial.Serial(self.port, baudrate=self.baudrate, xonxoff=self.xonxoff, timeout=self.timeout)
        vprint('Opened')

    def close(self):
        vprint('closing')
        self.serial.close()

    def flush(self):
        self.serial.flush()

    def seek_to_header_start(self):
        vprint('seeking to start')
        while True:
            # get two starting bytes
            c = self.serial.read(1)
            # print(c.hex(), end=' ', flush=True)
            if c == b'\x42':
                c2 = self.serial.read(1)
                # print(c2.hex(), end=' ', flush=True)
                if c2 != b'\x4d':
                    print('Not the start of the packet')
                    return True
                # print('',flush=True)
                return False

    def read_serial(self):
        vprint('Reading serial')

        # read from uart, up to 22 bytes
        data_uart = self.serial.readline(22)

        # print(fmt_packet(data_uart))
        # encode
        values = parse_packet(data_uart)
        if values is None:
            self.flush()
            return None

        # return data
        return values

    def write_serial(self, cmd, timeout=0.0):
        vprint('Writing serial')

        # writes binary data to the serial port
        write = self.serial.write(cmd)

        vprint('wrote {}'.format(prettyhex(cmd)))

        # set timeout
        time.sleep(timeout)

    def single_read(self):
        vprint('single read')

        while self.seek_to_header_start():
            vprint('seeking...')
        data = self.read_serial()

        # data read-out
        return data

    def check_read(self):
        # try to prompt a result
        self.cmd_active()
        single_data = self.single_read()
        if single_data is None:
            raise RuntimeError('Unable to get values from device')

        vprint(':) passed checkpoint :) ')

    def read_pm_once(self):

        try:
            single_data = [datetime.datetime.now().isoformat()]
            single_data += self.single_read()
            # print(single_data)
            time.sleep(2)
            if single_data:
                dd = dict(zip(PMColumns, single_data))
                if not dd['status']:
                    dd.pop('status')
                return dd
            else:
                vprint('data values were empty')
        except RuntimeError:
            print('break')

        return dict(zip(PMColumns, [datetime.datetime.now().isoformat(), np.nan, np.nan, np.nan, 'fail']))


    def read_pm(self):

        vprint('read_pm()')

        self.check_read()

        self.cmd_active()
        # self.cmd_passive()

        # measure pm n-times
        all_data = []

        # for i in range(1, n):
        while True:
            try:
                single_data = self.single_read()
                print(single_data)
                time.sleep(2)
                if single_data:
                    all_data.append(single_data)
                else:
                    vprint('data values were empty')
            except KeyboardInterrupt:
                print('break')
                break
        vprint('averaging')
        # reshape list into matrix
        data = np.array(all_data).reshape((-1, 3))

        # put the sensor in the passive mode
        # self.write_serial(Commands.passive)
        # data = self.single_read()
        # standby mode, aka. sleep


        # return averaged data
        return data

    def cmd_wakeup(self):
        """Tell device to wake up, enable fan"""
        vprint('wakeup')
        self.write_serial(Commands.wakeup, 1)

    def cmd_standby(self):
        """Put device into standby, disable fan"""
        vprint('standby')
        self.write_serial(Commands.standby)

    def cmd_passive(self):
        vprint('passive mode')
        self.write_serial(Commands.passive)

    def cmd_active(self):
        vprint('active mode')
        self.write_serial(Commands.active)

