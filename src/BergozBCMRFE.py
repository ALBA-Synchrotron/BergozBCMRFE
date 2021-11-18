# -*- coding: utf-8 -*-
#
# This file is part of tango-ds (https://www.tango-controls.org/)
#
# tango-ds is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# tango-ds is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with tango-ds.  If not, see <http://www.gnu.org/licenses/>.


"""
Bergoz BCMRFE front end device server
"""

# standard python imports
import collections
import struct
import sys
import threading
import time

# managed by bumpversion do not edit manually
__version = '1.0.1'


# Python tango imports (includes legacy support)
try:
    import tango  # python tango >= 8
    from tango.server import LatestDeviceImpl as DeviceImplementation
except Exception as e:
    import PyTango as tango  # python tango < 8
    from PyTango import Device_4Impl as DeviceImplementation


class Communicator(object):
    """
    Utility class to handle serial communications with the hardware
    """
    KNOWN_SERIALS = ['serial', 'Serial', 'PySerial']
    TERMINATOR = '\n\0'

    def __init__(self, device, *args, **kwargs):
        # find out if using a serial port (e.g. /dev/ttyACM0) or a tango device
        serial_class = None
        try:
            self.serial = tango.DeviceProxy(device)
            serial_class = self.serial.info().dev_class
        except:  # not a tango device
            pass
        if serial_class is None:
            import serial
            self.serial = serial.Serial(device, *args, **kwargs)
            serial_class = 'serial'
        if serial_class not in self.KNOWN_SERIALS:
            raise('Unknown serial. Valid ones are: %s' % self.KNOWN_SERIALS)

        # initialize serial device if necessary
        if serial_class == self.KNOWN_SERIALS[0]:  # serial port
            self._comm = self._communicate_raw
        elif serial_class == self.KNOWN_SERIALS[1]:  # Serial tango dev
            self.serial.command_inout('DevSerFlush', 2)
            self._comm = self._communicate_raw_Serial
        else:  # PySerial tango dev
            if self.serial.State() != tango.DevState.ON:
                self.serial.command_inout('Open')
            self.serial.command_inout('FlushInput')
            self.serial.command_inout('FlushOutput')
            self._comm = self._communicate_raw_PySerial

    def _communicate_raw(self, cmd='', output=False, strip=False):
        if cmd != '':
            cmd += self.TERMINATOR
            self.serial.write(cmd)
        if output:
            read_ = self.serial.readline()
            if strip:
                out = read_.strip()
            else:
                # preserve all characters except \n, \r and \x00
                out = filter(lambda x: x not in ('\r', '\n', '\x00'), read_)
            return out

    def _communicate_raw_Serial(self, cmd='', output=False, strip=False):
        if cmd != '':
            cmd += self.TERMINATOR
            self.serial.command_inout('DevSerWriteString', cmd)
        if output:  # output expected
            read_ = self.serial.command_inout('DevSerReadLine')
            if strip:
                out = read_.strip()
            else:
                # preserve all characters except \n, \r and \x00
                out = filter(lambda x: x not in ('\r', '\n', '\x00'), read_)
            return out

    def _communicate_raw_PySerial(self, cmd='', output=False, strip=False):
        if cmd != '':
            cmd += self.TERMINATOR
            self.serial.command_inout('Write', bytearray(cmd))
        if output:  # output expected
            read_ = self.serial.command_inout('ReadLine')
            if read_.size == 0:
                msg = ('Got empty return value from PySerial device. '
                       'This is probably a communication error. Please check')
                raise(msg)
            read_ = read_.tostring()
            if strip:
                out = read_.strip()
            else:
                # preserve all characters except \n, \r and \x00
                out = filter(lambda x: x not in ('\r', '\n', '\x00'), read_)
            return out


class BCMRFE(DeviceImplementation):
    """
    Bergoz BCMRFE front end device server
    """

    TIMEOUT = 3  # timeout in secs when waiting from instrument response
    MAX_SAMPLES = 2048  # max read samples to accumulate
    SEPARATOR = '='

    commands = {
        # List of all available commands. Note that not all them will be added
        # automatically (they require special treatment)
        # Note that these are the write commands: read commands are the same
        # but only using the first 2 characters and a quote mark (e.g. D0?)

        # Set on-board's digital delay line value in nanoseconds
        # "xx" must be an integer in HEX format within the range 00 to FF
        'DigitalDelay': 'D0:00xx',

        # Save BCM-RF-E configuration to microcontroller’s EEPROM
        # (this command requires special processing)
        'SaveConfig':   'E0:0001',

        # Set BCM-RF-E switch configuration
        # Single bits of "x" are used to switch modes:
        # Bit0 = 0 => use external trigger
        # Bit0 = 1 => use internal trigger
        # Bit1 = 0 => T-C mode
        # Bit1 = 1 => S-H mode
        # Bit2 = 0 => Internal clock off (for T-C mode)
        # Bit2 = 1 => Internal clock on (for S-H mode)
        # Bit3 = 0 => Digital Delay line on, Front panel trimmer off
        # Bit3 = 1 => Digital Delay line off, Front panel trimmer on
        # (this command requires special processing)
        'SwitchConfig': 'I0:000x',

        # Activate CAL-FO mode (optional)
        'CALFO': 'K0:000x',  # (x=1 on, x=0 off)

        # Activate microcontroller’s reverse function algorithm
        'ReverseAlgorithm': 'M0:000x',  # (x=1 on, x=0 off)

        # Read BCM-RF-E serial number S0? (read only)
        # (this command requires special processing)
        'SerialNumber': 'S0?',

        # Set number of ADC samples used for averaging
        'ADCSamples': 'T0:xxxx',  # "xxxx" integer in HEX "0000" to "FFFF"

        # Set calibration constant Qcal (picocoulombs) or Ical (microamperes)
        # The calibration constants are transmitted as hexa-decimal
        # representations of IEEE754 encoded 32bit floating point numbers. The
        # command frame V1:yyyy sends the upper 16bit. The command frame
        # V0:xxxx sends the lower 16bit. Example:
        # Decimal:
        #     Qcal = 0.015766
        # Binary of IEEE754 encoded 32bit float:
        #     Qcal = 00111100100000010010011110110011
        # HEX of IEEE754 encoded 32bit float:
        #     Qcal = 3C8127B3
        # The frames V1:3C81 and V0:27B3 need to be send.
        # For readback, see previous table for a description of the data format
        # Note that the response frame "V0" contains the upper 16bit and the
        # response frame "V1" contains the lower 16bit.
        # WARNING!!!!: thought it is not stated in the documentation it looks
        # like the readback is just the opposite way: V0:3C81 and V1:27B3
        # (this command requires special processing)
        'CalibrationConstant': 'V0:yyyy',  # and V1:xxxx

        # Set calibration constant Ucal (volts). See previous for description
        # (this command requires special processing)
        'UcalConstant': 'W0:yyyy',  # and V1:xxxx
    }

    class Updater(threading.Thread):
        """
        This class will continuously read ouput from the instrument and will
        also take care of handling command answers. The instrument is
        continuously providing sampled output (if in TC mode), but mixed with
        that continuous stream the answers to commands will also be included
        """

        def __init__(self, device):
            threading.Thread.__init__(self)
            self.device = device
            self.stop = False

        def run(self):
            while not self.stop:
                try:
                    ret = self.device.comm._comm(output=True)
                    if ret in (None, ''):
                        pass  # ignore (probably timeout ocurred)
                    elif ret[0] == 'A':  # TC or SH sample
                        # text in hexadecimal
                        self.device.sampled.append(
                            int(ret.split(BCMRFE.SEPARATOR)[1], 16))
                    elif ret[0] == '!':
                        pass  # SH mode: new trigger received (ignore)
                    else:
                        self.device.answers.append(ret)
                except Exception as e:
                    msg = 'Error reading from device: %s' % str(e)
                    self.device.error_stream(msg)

        def exit(self):
            self.stop = True

    # ==================================================================
    #   standard methods
    # ==================================================================

    def __init__(self, *args, **kwargs):
        DeviceImplementation.__init__(self, *args, **kwargs)
        self.debug_stream('In __init__()')
        # setup standard state machine
        self.setup_state_machine()
        # initialize communicator
        self.comm = None
        # sampled data from instrument: the instrument is continuously sending
        # the sampled values from the adc
        self.sampled = collections.deque(maxlen=self.MAX_SAMPLES)
        # answers from instrument to requested data (this does not include
        # sampled values)
        self.answers = collections.deque(maxlen=2)  # answers from instrument
        BCMRFE.init_device(self)

    def init_device(self):
        try:
            self.debug_stream('In init_device()')
            self.get_device_properties(self.get_device_class())
            self.sampled.clear()
            self.answers.clear()
            if self.comm is None:
                self.SerialDev = list(self.SerialDev)
                for serial in self.SerialDev:
                    try:
                        self.comm = Communicator(serial, timeout=self.Timeout)
                        break  # comm was correctly initialized
                    except Exception as e:
                        self.comm = None
                        self.error_stream(
                            'Init failed for %s: %s' % (serial, str(e)))
                else:
                    raise  # No sucess initializing comm
            msg = 'System seems OK'
            self.info_stream(msg)
            self._set_state(tango.DevState.ON, msg, force_init=True)
            self.updater = self.Updater(self)
            self.updater.start()
        except Exception as e:
            msg = 'Error while initialing system: %s' % str(e)
            self.error_stream('%s' % msg)
            self._set_state(tango.DevState.FAULT, msg, force_init=True)

    def always_executed_hook(self):
        self.debug_stream('In always_excuted_hook()')

    def delete_device(self):
        self.debug_stream('In delete_device()')
        del self.comm
        self.comm = None
        if self.updater.is_alive():
            self.debug_stream('updater canceling')
            self.updater.exit()
            self.debug_stream('updater cancelled')
            self.debug_stream('updater joining')
            self.updater.join(10)
            if self.updater.is_alive():
                msg = 'updater joining timed out !!!'
            else:
                msg = 'updater correctly joined'
            self.debug_stream(msg)

    # ==================================================================
    #   utility methods
    # ==================================================================

    def _set_state(self, new_state, new_status=None, force_init=False):
        state_now = self.get_state()
        # don't allow to change FAULT state unless initialing
        nok = [tango.DevState.FAULT]
        if state_now in nok and not force_init:
            return
        if (state_now == new_state) and (not force_init):
            return
        self.set_state(new_state)
        if new_status is not None:
            self.set_status(new_status)

    def _is_attr_allowed_default(self, req_type):
        if ((req_type == tango.AttReqType.WRITE_REQ) and
                (self.get_state() != tango.DevState.ON)):
            return False
        return True

    def _is_cmd_allowed_default(self):
        if self.get_state() != tango.DevState.ON:
            return False
        return True

    def setup_state_machine(self):
        """Setup standard state machine for Attributes and Commands"""
        for attr in BCMRFEClass.attr_list:
            method = 'is_' + attr + '_allowed'
            if not hasattr(BCMRFE, method):
                setattr(BCMRFE, method, self._is_attr_allowed_default)
        for cmd in BCMRFEClass.cmd_list:
            method = 'is_' + cmd + '_allowed'
            if not hasattr(BCMRFE, method):
                setattr(BCMRFE, method, self._is_cmd_allowed_default)

    def welcome(self, attr, mode='read_'):
        if attr is None:
            name = ''
        else:
            name = attr.get_name()
        msg = 'In %s::%s%s()' % (self.get_name(), mode, name)
        self.info_stream(msg)

    def read_standard(self, cmd, data_type=int):
        if cmd.find('?') < 0:
            cmd = '%s?' % cmd.split(':')[0]
        # Do not try communication if state if FAULT
        if self.get_state() == tango.DevState.FAULT:
            return None
        # send command
        self.comm._comm(cmd=cmd)
        # wait for response: updater thread will be running continuously
        timeout = False
        start = time.time()
        returned = ''
        while len(self.answers) < 1 and not timeout:
            time.sleep(0.1)
            timeout = (time.time() - start > self.TIMEOUT)
        if not timeout:
            returned = self.answers.popleft()
        if timeout or returned == '':
            self.answers.clear()  # some problem happened: clear answers
            raise Exception('No answer got from instrument')
        if data_type is int:
            # answers are text in hexadecimal
            value = int(returned.split(self.SEPARATOR)[1], 16)
        else:
            # return the string as got (requestor will process it)
            value = returned.split(self.SEPARATOR)[1]
        return value

    def read_standard_attr(self, attr):
        cmd = self.commands[attr.get_name()]
        try:
            value = self.read_standard(cmd)
            if value is None:
                attr.set_quality(tango.AttrQuality.ATTR_INVALID)
            else:
                attr.set_value(value)
        except Exception as e:
            attr.set_quality(tango.AttrQuality.ATTR_INVALID)
            msg = 'Error reading data from device (check comm and Init)'
            self.error_stream('%s: %s' % (msg, str(e)))
            self._set_state(tango.DevState.FAULT, msg)

    def write_standard(self, cmd, value):
        hex_digits = cmd.count('x')
        if hex_digits > 0:
            # convert to string hex
            formatter = formatter = '%0.' + str(hex_digits) + 'X'
            str_hex = formatter % value
            cmd = cmd.replace('x', '')  # remove x symbols from command
            cmd += str_hex  # add string hex
        try:
            # send command (no way to check correction unless reading back)
            self.comm._comm(cmd=cmd)
        except Exception as e:
            msg = 'Error writing data to device (check comm and Init)'
            self.error_stream('%s: %s' % (msg, str(e)))
            self._set_state(tango.DevState.FAULT, msg)
            raise

    def write_standard_attr(self, attr):
        cmd = self.commands[attr.get_name()]
        value = attr.get_write_value()
        self.write_standard(cmd, value)

    def enable_switch_bit(self, enable, bit):
        cmd = self.commands['SwitchConfig']
        try:
            current_value = self.read_standard(cmd)
            if current_value is None:
                raise Exception('Unable to communicate')
            if enable == 1:
                value = current_value | (1 << bit)
            else:
                value = current_value & ~(1 << bit)
            self.write_standard(cmd, value)
        except Exception as e:
            msg = 'Error setting flag in device (check comm and Init)'
            self.error_stream('%s: %s' % (msg, str(e)))
            self._set_state(tango.DevState.FAULT, msg)
            raise

    # ==================================================================
    #   command methods
    # ==================================================================

    def SaveConfig(self):
        """Save instrument's configuration in its own EEPROM"""
        self.welcome(attr=None, mode='SaveConfig')
        self.comm._comm(cmd=self.commands['SaveConfig'])

    # ==================================================================
    #   read/write attribute methods
    # ==================================================================

    def read_ADCSamples(self, attr):
        self.welcome(attr)
        self.read_standard_attr(attr)

    def write_ADCSamples(self, attr):
        self.welcome(attr, mode='write_')
        self.write_standard_attr(attr)

    def read_CALFO(self, attr):
        self.welcome(attr)
        self.read_standard_attr(attr)

    def write_CALFO(self, attr):
        self.welcome(attr, mode='write_')
        self.write_standard_attr(attr)

    def read_CalibrationConstant(self, attr):
        self.welcome(attr)
        hex_val = ''
        cmd = self.commands[attr.get_name()]
        for i in range(2):  # 2 answers (see CalibrationConstant above)
            if i != 0:
                cmd = ''  # request command must be sent only once
            value = self.read_standard(cmd, data_type=float)
            if value is None:
                attr.set_quality(tango.AttrQuality.ATTR_INVALID)
                return
            hex_val = value[4:] + hex_val  # first 4 chars are always 0
        float_value = struct.unpack('>f', hex_val.decode('hex'))[0]
        attr.set_value(float_value)

    def write_CalibrationConstant(self, attr):
        self.welcome(attr, mode='write_')
        value = attr.get_write_value()
        hex_value = hex(struct.unpack('>I', struct.pack('>f', value))[0])
        hex_value = hex_value.upper().strip('0X')
        write_cmd = self.commands[attr.get_name()][0]
        for i, value in zip([1, 0], [hex_value[4:], hex_value[:4]]):
            cmd = '%s%d:%s' % (write_cmd, i, value)
            self.write_standard(cmd, value)

    def read_DigitalDelay(self, attr):
        self.welcome(attr)
        self.read_standard_attr(attr)

    def write_DigitalDelay(self, attr):
        self.welcome(attr, mode='write_')
        self.write_standard_attr(attr)

    def read_ReverseAlgorithm(self, attr):
        self.welcome(attr)
        self.read_standard_attr(attr)

    def write_ReverseAlgorithm(self, attr):
        self.welcome(attr, mode='write_')
        self.write_standard_attr(attr)

    def read_SampledSignal(self, attr):
        self.welcome(attr)
        # return last read value if any
        if len(self.sampled) > 0:
            attr.set_value(self.sampled[-1] / 1e3)
        else:
            attr.set_quality(tango.AttrQuality.ATTR_INVALID)

    def read_SampledSignalBuffer(self, attr):
        self.welcome(attr)
        attr.set_value(self.sampled)

    def read_SerialNumber(self, attr):
        self.welcome(attr)
        cmd = self.commands[attr.get_name()]
        value = self.read_standard(cmd, str)
        if value is None:
            attr.set_quality(tango.AttrQuality.ATTR_INVALID)
        else:
            attr.set_value(value)

    def read_SwitchConfig(self, attr):
        self.welcome(attr)
        cmd = self.commands[attr.get_name()]
        value = self.read_standard(cmd)
        if value is None:
            attr.set_quality(tango.AttrQuality.ATTR_INVALID)
        else:
            attr.set_value(value)

    def write_SwitchConfig(self, attr):
        self.welcome(attr, mode='write_')
        cmd = self.commands[attr.get_name()]
        value = attr.get_write_value()
        self.write_standard(cmd, value)
        self.sampled.clear()  # clear samples received (mode may be changed)

    def read_SwitchInternalTrigger(self, attr):
        self.welcome(attr)
        cmd = self.commands['SwitchConfig']
        value = self.read_standard(cmd)
        if value is None:
            attr.set_quality(tango.AttrQuality.ATTR_INVALID)
        else:
            attr.set_value(bool(value & 0b0001))

    def write_SwitchInternalTrigger(self, attr):
        self.welcome(attr, mode='write_')
        # enable/disable bit 0 in SwitchConfig register
        self.enable_switch_bit(int(attr.get_write_value()), 0)

    def read_SwitchModeSH(self, attr):
        self.welcome(attr)
        cmd = self.commands['SwitchConfig']
        value = self.read_standard(cmd)
        if value is None:
            attr.set_quality(tango.AttrQuality.ATTR_INVALID)
        else:
            attr.set_value(bool(value & 0b0010))

    def write_SwitchModeSH(self, attr):
        self.welcome(attr, mode='write_')
        # enable/disable bit 1 in SwitchConfig register
        self.enable_switch_bit(int(attr.get_write_value()), 1)
        self.sampled.clear()  # clear samples received

    def read_SwitchInternalClock(self, attr):
        self.welcome(attr)
        cmd = self.commands['SwitchConfig']
        value = self.read_standard(cmd)
        if value is None:
            attr.set_quality(tango.AttrQuality.ATTR_INVALID)
        else:
            attr.set_value(bool(value & 0b0100))

    def write_SwitchInternalClock(self, attr):
        self.welcome(attr, mode='write_')
        # enable/disable bit 2 in SwitchConfig register
        self.enable_switch_bit(int(attr.get_write_value()), 2)

    def read_SwitchDigitalDelayOff(self, attr):
        self.welcome(attr)
        cmd = self.commands['SwitchConfig']
        value = self.read_standard(cmd)
        if value is None:
            attr.set_quality(tango.AttrQuality.ATTR_INVALID)
        else:
            attr.set_value(bool(value & 0b1000))

    def write_SwitchDigitalDelayOff(self, attr):
        self.welcome(attr, mode='write_')
        # enable/disable bit 3 in Swi hConfig register
        self.enable_switch_bit(int(attr.get_write_value()), 3)

    def read_UcalConstant(self, attr):
        self.welcome(attr)
        hex_val = ''
        cmd = self.commands[attr.get_name()]
        for i in range(2):  # 2 answers (see CalibrationConstant above)
            if i != 0:
                cmd = ''  # request command must be sent only once
            value = self.read_standard(cmd, data_type=float)
            if value is None:
                attr.set_quality(tango.AttrQuality.ATTR_INVALID)
                return
            hex_val = value[4:] + hex_val  # first 4 chars are always 0
        float_value = struct.unpack('>f', hex_val.decode('hex'))[0]
        attr.set_value(float_value)

    def write_UcalConstant(self, attr):
        self.welcome(attr, mode='write_')
        value = attr.get_write_value()
        hex_val = hex(struct.unpack('>I', struct.pack('>f', value))[0])
        hex_val = hex_val.upper().strip('0X')
        write_cmd = self.commands[attr.get_name()][0]
        for i, value in zip([1, 0], [hex_val[:4], hex_val[4:]]):
            cmd = '%s%d:%s' % (write_cmd, i, value)
            self.write_standard(cmd, value)


class BCMRFEClass(tango.DeviceClass):

    # Class Properties
    class_property_list = {
    }

    # Device Properties
    device_property_list = {
        'SerialDev':
            [tango.DevVarStringArray,
             'List of device names to connect to. It may contain a single '
             'value or different values. In case more than one values is '
             'provided communication will be tried with all them before '
             'giving up. We have observed that when the instrument is '
             'rebooted and/or the usb cable is plugged/unplugged the os will '
             'switch the serial dev name between ttyACM0 and ttyACM1. The '
             'entries may be a system serial device (e.g. /dev/ttyACM0) or a '
             'tango serial device (e.g. SR/DI/SerialBergoz)',
             # default value
             ['/dev/ttyACM0', '/dev/ttyACM1']],
        'BaudRate':
            [tango.DevShort,
             'Baudrate (used only if using a system serial)',
             9600],
        'ByteSize':
            [tango.DevShort,
             'Bytesize (used only if using a system serial)',
             8],
        'Parity':
            [tango.DevString,
             'Parity (used only if using a system serial)',
             'N'],
        'StopBits':
            [tango.DevShort,
             'StopBits (used only if using a system serial)',
             1],
        'Timeout':
            [tango.DevShort,
             'Timeout (secs if system serial, not used otherwise)',
             3],
    }

    # Command definitions
    cmd_list = {
        # Save instrument's configuration in its own EEPROM
        'SaveConfig':
            [[tango.DevVoid, ''],
             [tango.DevVoid, '']],
    }

    # Attribute definitions
    attr_list = {
        'ADCSamples':
            [[tango.DevUShort, tango.SCALAR, tango.READ_WRITE],
             {
                'label': 'ADC samples used for averaging',
                'min value': 0x0,
                'max value': 0xFFFF,
            }],
        'CALFO':
            [[tango.DevBoolean, tango.SCALAR, tango.READ_WRITE],
             {
                'label': 'CAL-FO mode enable/disable',
            }],
        'CalibrationConstant':
            [[tango.DevFloat, tango.SCALAR, tango.READ_WRITE],
             {
                'label': 'Qcal (picocoulombs) or Ical (microamperes)',
            }],
        'DigitalDelay':
            [[tango.DevUChar, tango.SCALAR, tango.READ_WRITE],
             {
                'label': 'Digital delay',
                'unit': 'nanoseconds',
                'min value': 0x0,
                'max value': 0xFF,
            }],
        'ReverseAlgorithm':
            [[tango.DevBoolean, tango.SCALAR, tango.READ_WRITE],
             {
                'label': 'Reverse algorithm mode enable/disable',
            }],
        'SampledSignal':
            [[tango.DevFloat, tango.SCALAR, tango.READ],
             {
                'label': 'Sampled signal',
                'unit': 'uvolt'
            }],
        'SampledSignalBuffer':
            [[tango.DevLong, tango.SPECTRUM, tango.READ, BCMRFE.MAX_SAMPLES],
             {
                'label': 'Sampled signal buffer',
                'unit': 'uvolt'
            }],
        'SerialNumber':
            [[tango.DevString, tango.SCALAR, tango.READ],
             {
                'label': 'Serial number',
            }],
        'SwitchConfig':
            [[tango.DevUChar, tango.SCALAR, tango.READ_WRITE],
             {
                'label': 'Switches configuration',
                'min value': 0x0,
                'max value': 0xF,
            }],
        'SwitchInternalTrigger':
            [[tango.DevBoolean, tango.SCALAR, tango.READ_WRITE],
             {
                'label': 'Internal trigger on (external used if off)',
            }],
        'SwitchModeSH':
            [[tango.DevBoolean, tango.SCALAR, tango.READ_WRITE],
             {
                'label': 'SH mode on or off (TC if off)',
            }],
        'SwitchInternalClock':
            [[tango.DevBoolean, tango.SCALAR, tango.READ_WRITE],
             {
                'label': 'Internal clock off (for TC) or on (for SH)',
            }],
        'SwitchDigitalDelayOff':
            [[tango.DevBoolean, tango.SCALAR, tango.READ_WRITE],
             {
                'label': 'Digital delay off (on if disabled)',
            }],
        'UcalConstant':
            [[tango.DevFloat, tango.SCALAR, tango.READ_WRITE],
             {
                'label': 'calibration constant Ucal',
                'unit': 'volt'
            }],
    }


def main(*args, **kwargs):
    try:
        tango_util = tango.Util(sys.argv)
        if tango.Release.version_number <= 711:
            tango_util.add_TgClass(BCMRFEClass, BCMRFE, 'BCMRFE')
        else:
            tango_util.add_class(BCMRFEClass, BCMRFE)
        util_instance = tango.Util.instance()
        util_instance.server_init()
        util_instance.server_run()
    except tango.DevFailed as e:
        print('-------> Received a DevFailed exception:', str(e))
    except Exception as e:
        print('-------> An unforeseen exception occurred....', str(e))


if __name__ == '__main__':
    main()
