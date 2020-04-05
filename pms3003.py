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
from util import TxnObj, LitePstruct


FLUSH=False

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

PMColumns = ['time', 'pm1', 'pm2_5', 'pm10', 'status']

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
    return '{: >3} >{} | {:04x}'.format(len(bytes), prettyhex(bytes), calc_checksum(bytes[:-cs_len]))


class PMModel(object):
    pass


class PMResult(LitePstruct):
    __slots__ = ['pm1', 'pm2_5', 'pm10', 'time', 'status']

    def __init__(self, pm1=np.nan, pm2_5=np.nan, pm10=np.nan, time=None, status=None):
        if time is None:
            time = datetime.datetime.now().isoformat()
        if status is None:
            status = ''
        super(PMResult, self).__init__(pm1=pm1, pm2_5=pm2_5, pm10=pm10, time=time, status=status)

    @classmethod
    def parse_tuple(cls, *args, status=None):
        if len(args) < 3:
            raise ValueError('Could not parse PM tuple, not enough arguments: {}'.format(args))
        return PMResult(args[0], args[1], args[2], status=status)


def parse_packet(packet: bytes, cs_len=2) -> PMResult:
    vprint(fmt_packet(packet))
    start, framelen = struct.unpack('>HH', packet[:4])
    if start != 0x424d:
        print('sentinel byte mismatch: {}'.format(fmt_packet(packet)))
        return PMResult(status='failed parse')

    vprint('payload length: {}'.format(framelen))

    if framelen != 20:
        print('Unexpected payload size: {}'.format(framelen))

    try:
        payload_raw = packet[4:-cs_len]
        payload = struct.unpack('>{}H'.format(len(payload_raw)//2), payload_raw)
        return PMResult.parse_tuple(*payload)
    except Exception as e:
        print(type(e).__name__, e)
        return PMResult(status='failed parse')


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
        self.last_bytes = b''
        self.buffer = b''

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
        vprint('flushing')
        self.serial.flush()

    def read(self, size=1):
        """Read serial into the local buffer """
        self.buffer += self.serial.read(size=size)

    def dump(self, end='\n', flush=FLUSH):
        """Dump the contents of the buffer to screen for diagnostic"""
        print(prettyhex(self.buffer), end=end, flush=flush)
        self.clear_buffer()

    def clear_buffer(self):
        self.buffer = b''

    def seek_to_header_start(self):
        """deprecated"""
        vprint('seeking to start')
        while True:
            # get two starting bytes
            c = self.serial.read(1)
            # print(c.hex(), end=' ', flush=FLUSH)
            if c == b'\x42':
                c2 = self.serial.read(1)
                # print(c2.hex(), end=' ', flush=FLUSH)
                if c2 != b'\x4d':
                    print('Not the start of the packet')
                    return True
                # print('',flush=FLUSH)
                return False

    def sync(self):
        vprint('seeking to start')
        self.m.desync()
        self.dump()
        while True:
            # get two starting bytes
            self.read()
            if self.buffer != b'\x42':
                self.dump(end=' ')
                continue
            self.read()
            # print(c2.hex(), end=' ', flush=FLUSH)
            if self.buffer != b'\x42\x4d':
                self.dump()
                print('Not the start of the packet, restarting')  # this is rare
                continue
            print('',flush=FLUSH)

            # we should be at the start of the packet. Read and throw away so we can sync up
            self.m.wait()
            self.read(2)
            start, framelen = struct.unpack('>HH', self.buffer[:4])
            if framelen > 20:
                print('Framelen is abnormally high: {}'.format(framelen))
            if framelen > 64:
                print('wacky framelen, resyncing')
                self.dump()
                self.m.desync()
                continue
            self.read(framelen)
            self.last_bytes = self.buffer
            self.clear_buffer()
            print(fmt_packet(self.last_bytes))
            self.m.sync()
            return self.last_bytes

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

    def read_packet(self, cs_len=2) -> PMResult:
        vprint('Reading serial')
        if self.m.state not in states_synced:
            raise ValueError('frame is not synced: {}'.format(self.m.state))

        # self.read(4)
        header = self.serial.read(4)
        # header = self.buffer
        start, framelen = struct.unpack('>HH', header[:4])
        if start != 0x424d:
            print(prettyhex(self.last_bytes + header))
            print('sentinel byte mismatch')
            self.m.desync()
            return PMResult(status='sentinel byte mismatch')

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
            errfmt = '{}: {}'.format(e.__class__.__name__, e)
            print(errfmt)
            return PMResult(status=errfmt)

        packet = header + payload_plus_cs
        vprint(fmt_packet(packet))

        self.last_bytes = header + payload_plus_cs
        if len(payload) < 3:
            self.flush()
            self.m.desync()
            print('payload too short: {}'.format(payload))
            return PMResult(status='payload too short')

        if cs != actual_cs:
            print('checksum mismatch: wanted {:02x} got {:02x}'.format(cs, actual_cs))
            self.m.desync()
        else:
            self.m.sync()

        return PMResult.parse_tuple(*payload, status=self.m.state)

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

    def gen_read(self):
        if self.m.state not in states_opened:
            raise ValueError('device is not opened: {}'.format(self.m.state))

        while True:
            while self.m.state not in states_synced:
                print('Current state: {}'.format(self.m.state))
                packet = self.sync()
                yield parse_packet(packet)

            yield self.read_packet()


    def sm_read_active_once(self) -> dict:
        if self.m.state not in states_opened:
            raise ValueError('device is not opened: {}'.format(self.m.state))

        while self.m.state not in states_synced:
            print('Current state: {}'.format(self.m.state))
            self.sync()

        try:
            data_dict = self.read_packet()
            if data_dict:
                data_dict.update({'time': datetime.datetime.now().isoformat()})
                return data_dict
            else:
                vprint('data values were empty')
                return dict(zip(PMColumns, [datetime.datetime.now().isoformat(), np.nan, np.nan, np.nan, self.m.state]))

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

