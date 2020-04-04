#!bin/python
# -*- coding: utf-8 -*-
"""PMS3003

PMS3003 is a kind of digital and universal particle concentration sensor, which can be used to obtain the number of
suspended particles in the air, i.e. the concentration of particles, and output them in the form of digital interface.
This sensor can be inserted into variable instruments related to the concentration of suspended particles in the air or
other environmental improvement equipments to provide correct concentration data in time.

Mainly output is mass concentration and the unit is μ g/m3.
There are two options for digital output: passive and active.
Default mode is active after power up. In this mode sensor would send serial data to the host automatically.
The active mode is divided into two sub-modes: stable mode and fast mode. If the concentration change is small the
sensor would run at stable mode with the real interval of 2.3s. And if the change is big the sensor would be changed
to fast mode automatically with the interval of 200~800ms, the higher of the concentration, the shorter of the interval.
"""

import serial
import time
import datetime
import struct
import numpy as np
from transitions import Machine, Transition, State
from vprint import vprint
from util import TxnObj


pms3003_states = ['closed',     # serial port is closed
                  'opened',     # serial port is open but we don't know if enabled yet
                  'awake',      # serial port is open and device is enabled but active/passive is not know yet
                  'sleeping',   # serial port is open but device is in sleep
                  'a_desynced',     # awake, active, but we don't know where in the stream we are
                  'a_synced',       # awake, active, and we know we are at a 24 byte boundary, expected to start with BM
                  'a_waiting',      # awake, active, waiting on next packet
                  'p_desynced',  # awake, passive, but we don't know where in the stream we are
                  'p_synced',    # awake, passive, and we know we are at a 24 byte boundary, expected to start with BM
                  'p_waiting',   # awake, passive, waiting on next packet
                  ]

# todo: Hierarchical state machine
states_active = ['a_desynced', 'a_synced', 'a_waiting']
states_passive = ['p_desynced', 'p_synced', 'p_waiting']
states_synced = ['a_synced', 'p_synced']
states_unknwn = ['opened', 'awake', 'sleeping']
states_opened = states_unknwn + states_active + states_passive


pms3003_transitions = [
    TxnObj('open', 'closed', 'opened'),        # opening serial port
    TxnObj('close', pms3003_states, 'closed'),
    TxnObj('wakeup', 'opened', 'awake'),
    TxnObj('sleep', states_opened, 'sleeping'),  # maybe this should only be from synced states
    TxnObj('activate', states_passive + states_unknwn, 'a_desynced'),
    TxnObj('passivate', states_active + states_unknwn, 'p_desynced'),
    TxnObj('sync', states_active, 'a_synced'),
    TxnObj('sync', states_passive, 'p_synced'),
    TxnObj('wait', states_active, 'a_waiting'),
    TxnObj('wait', states_passive, 'p_waiting'),
    TxnObj('desync', states_active, 'a_desynced'),
    TxnObj('desync', states_passive, 'p_desynced'),
]

# todo: there's gotta be an automatic way to do this
class PMModel(object):
    pass
    # def __init__(self):
    #     pass
    #     self.state = ''
    #
    # def to_opened(self):
    #     pass
    #
    # def to_closed(self):
    #     pass
    #
    # def open(self):
    #     pass
    #
    # def close(self):
    #     pass
    #
    # def sync(self):
    #     pass
    #
    # def desync(self):
    #     pass
    #
    # def wait(self):
    #     pass
    #
    # def wakeup(self):
    #     pass
    #
    # def sleep(self):
    #     pass
    #
    # def activate(self):
    #     pass
    #
    # def passivate(self):
    #     pass

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


def calc_checksum(bytes: bytes):
    return sum(bytes) % 65536


def fmt_packet(bytes: bytes, delim: str = ' ', cs_len=2):
    return '{: >3} >{} | {:02x}'.format(len(bytes), prettyhex(bytes), calc_checksum(bytes[-cs_len:]))


def parse_packet(packet: bytes, standard=0, cs_len=2) -> list:
    vprint(fmt_packet(packet))
    start, framelen = struct.unpack('>HH', packet[:4])
    if start != 0x424d:
        print('sentinel byte mismatch: {}'.format(fmt_packet(packet)))
    vprint('payload length: {}'.format(framelen))

    if framelen != 20:
        print('Unexpected payload size: {}'.format(framelen))

    try:
        payload_raw = packet[4:-cs_len]
        payload = struct.unpack('>{}H'.format(len(payload_raw)//2), payload_raw)
        print(payload)
    except Exception as e:
        print(type(e).__name__, e)
    # data_raw = struct.unpack
    data_hex = packet[2:].hex()

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

        self.m = PMModel()
        self.machine = Machine(self.m, states=pms3003_states, transitions=pms3003_transitions, initial='closed')

    def __enter__(self):
        self.open()
        self.wakeup()
        self.activate()
        return self

    def __exit__(self, type, value, traceback):
        self.flush()
        # self.sleep()
        self.close()

    def open(self):
        vprint('Opening port {} @ {} ...'.format(self.port, self.baudrate))
        # open serial port
        self.serial = serial.Serial(self.port, baudrate=self.baudrate, xonxoff=self.xonxoff, timeout=self.timeout)
        self.m.open()
        vprint('Opened: {}'.format(self.m.state))

    def close(self):
        vprint('closing')
        self.serial.close()
        self.m.close()

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

    def sync(self):
        vprint('seeking to start')
        self.m.desync()
        while True:
            # get two starting bytes
            packet = self.serial.read(1)
            if packet != b'\x42':
                print(packet.hex(), end=' ', flush=True)
                continue
            packet += self.serial.read(1)
            # print(c2.hex(), end=' ', flush=True)
            if packet != b'\x42\x4d':
                print('Not the start of the packet')
                return False
            # print('',flush=True)

            # we should be at the start of the packet. Read and throw away so we can sync up
            self.m.wait()
            packet += self.serial.read(22)
            self.m.sync()
            return True

    def read_serial(self):
        vprint('Reading serial')

        # read from uart, up to 22 bytes
        data_uart = self.serial.read(22)

        # print(fmt_packet(data_uart))
        # encode
        values = parse_packet(b'BM' + data_uart)
        if values is None:
            self.flush()
            return None

        # return data
        return values

    def read_packet(self, cs_len=2):
        vprint('Reading serial')
        if self.m.state not in states_synced:
            raise ValueError('frame is not synced: {}'.format(self.m.state))

        header = self.serial.read(4)

        start, framelen = struct.unpack('>HH', header[:4])
        if start != 0x424d:
            print('sentinel byte mismatch: {}'.format(fmt_packet(header)))
            self.m.desync()
            return None

        vprint('payload length: {}'.format(framelen))

        if framelen != 20:
            print('Unexpected payload size: {}'.format(framelen))

        self.m.wait()

        payload_plus_cs = self.serial.read(framelen)
        cs = struct.unpack('>H', payload_plus_cs[-cs_len:])[0]
        actual_cs = calc_checksum(header + payload_plus_cs[:-cs_len])


        try:
            payload_raw = payload_plus_cs[:-cs_len]
            payload = list(struct.unpack('>{}H'.format(len(payload_raw) // 2), payload_raw))
            vprint(payload)
        except Exception as e:
            print(type(e).__name__, e)

        packet = header + payload_plus_cs
        vprint(fmt_packet(packet))
        # values = parse_packet(packet)
        values = payload[:3]
        if not values:
            self.flush()
            self.m.desync()
            return None

        if cs != actual_cs:
            print('checksum mismatch: wanted {:02x} got {:02x}'.format(cs, actual_cs))

        self.m.sync()

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
        self.activate()
        single_data = self.single_read()
        if single_data is None:
            raise RuntimeError('Unable to get values from device')

        vprint(':) passed checkpoint :) ')

    def sm_read_active_once(self):
        if self.m.state not in states_opened:
            raise ValueError('device is not opened: {}'.format(self.m.state))

        while self.m.state not in states_synced:
            print('Current state: {}'.format(self.m.state))
            self.sync()

        try:
            single_data = [datetime.datetime.now().isoformat()]
            data_tup = self.read_packet()
            if data_tup:
                dd = dict(zip(PMColumns, single_data + data_tup))
                if not dd.get('status', None):
                    dd.pop('status', None)
                return dd
            else:
                vprint('data values were empty')
        except RuntimeError:
            print('break')

        return dict(zip(PMColumns, [datetime.datetime.now().isoformat(), np.nan, np.nan, np.nan, 'fail']))

    def read_pm_once(self):
        """deprecated, will be subsumed by sm_read_active_once"""
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
        """deprecated"""
        vprint('read_pm()')

        self.check_read()

        self.activate()
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

    def wakeup(self):
        """Tell device to wake up, enable fan"""
        vprint('wakeup')
        self.write_serial(Commands.wakeup, 1)
        self.m.wakeup()

    def sleep(self):
        """Put device into standby, disable fan"""
        vprint('standby')
        self.write_serial(Commands.standby)

    def passivate(self):
        vprint('passive mode')
        self.write_serial(Commands.passive)
        self.m.passivate()

    def activate(self):
        vprint('active mode')
        self.write_serial(Commands.active)
        self.m.activate()

