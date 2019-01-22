import binascii
import configparser
import json
import logging
import queue
import threading
from enum import Enum

import serial

from .util import ascii_hex_to_string, calculate_crc


class Response(Enum):
    """Serial message types."""
    EMS = 0xF4
    GATEWAY = 0xA0


class GatewayReponse(Enum):
    """Gateway response message types."""
    SUCCESS = 0
    FAILURE = 1
    INVALID = 2
    CRC_ERROR = 3
    DATA_ERROR = 4
    REQUEST_NOT_CONFIRMED = 5
    REQUEST_NOT_VERIFIED = 6
    SEND_TIMEOUT = 7
    SEND_VERIFICATION_ERROR = 8
    UNSPECIFIED_ERROR = 9


class EmsError(Exception):
    """EMS Error exception subclass."""
    pass


class Ems:
    """PyEMS class for accessing the EMS bus via Arduino gateway."""

    def __init__(self,
                 config_filename='pyems/assets/default.cfg',
                 decoding_table_filename='pyems/assets/decoding_table.json',
                 open_port=True):
        """Initialize port, load config and decoding table."""
        self._port = serial.Serial()
        self._is_open = False
        self._read_queue = queue.Queue()

        self._load_config(config_filename)
        self._load_decoding_table(decoding_table_filename)

        if open_port:
            self.open()

    def __del__(self):
        """Destroy the current EMS instance."""
        self.close()

    def _create_logger(self, log_level, log_format, date_format):
        """Create an internal logger instance."""
        self._logger = logging.getLogger('pyems_logger')
        self._logger.setLevel(log_level)

        handler = logging.StreamHandler()
        handler.setLevel(log_level)

        formatter = logging.Formatter(fmt=log_format, datefmt=date_format)
        handler.setFormatter(formatter)

        self._logger.addHandler(handler)
        self._logger.propagate = False

    def _load_config(self, filename):
        """Load configuration file, initialize logger and port."""
        self._config = configparser.ConfigParser()
        try:
            with open(filename, 'r') as f:
                self._config.read_file(f)

            self._port.port = self._config['serial']['port_name']
            self._port.baudrate = int(self._config['serial']['baud_rate'])

            level = self._config['logging_ems']['level']
            log_format = self._config.get(
                'logging_ems', 'log_format', raw=True)
            date_format = self._config.get(
                'logging_ems', 'date_format', raw=True)
            self._create_logger(level, log_format, date_format)

            self._logger.debug('Config loaded.')
        except Exception as e:
            if self._logger:
                self._logger.critical('Failed to load EMS config file.')
            else:
                print('Failed to load EMS config file.')

            raise e

    def _load_decoding_table(self, filename):
        """Load the EMS frame decoding table."""
        try:
            with open(filename, encoding='utf-8') as f:
                self._decoding_table = json.load(f)
            self._logger.debug('Decoding table loaded.')
        except Exception as e:
            self._logger.critical('Failed to load the decoding table.')
            raise e

    def _decode_value(self, frame, data, offset):
        """Decode one value from a frame on a given offset. If the value type is 'f' (flag), return array of decoded flags."""
        str_offset = str(offset)
        frame_offsets = frame['offsets']
        decoded_values = []

        # get value's byte-length from the decoding table, 1 otherwise
        if 'length' in frame_offsets[str_offset]:
            value_len = int(frame_offsets[str_offset]['length'])
        else:
            value_len = 1

        # get value's type (number, flags, string)
        if 'type' in frame_offsets[str_offset]:
            item_type = frame_offsets[str_offset]['type']
        else:
            item_type = 'n'

        value_buf = data[offset:offset + value_len]
        if item_type == 'n':  # number
            # check unsigned attribute
            if 'unsigned' in frame_offsets[str_offset]:
                unsigned = frame_offsets[str_offset]['unsigned']
            else:
                unsigned = False

            # decode the value from bytes
            value = int.from_bytes(value_buf, 'big', signed=not unsigned)
            value_unsigned = int.from_bytes(value_buf, 'big')
            if 'mask' in frame_offsets[str_offset]:
                mask = int(frame_offsets[str_offset]['mask'], 0)
                value = value & mask

            if 'missing' in frame_offsets[str_offset]:
                missing = int(frame_offsets[str_offset]['missing'], 0)
                if value_unsigned == missing:
                    value = None

            if 'divisor' in frame_offsets[str_offset] and value != None:
                divisor = float(frame_offsets[str_offset]['divisor'])
                value = value / divisor

            decoded_values.append([frame_offsets[str_offset], value])
        elif item_type == 'f':  # flags
            for bit in range(8):
                str_bit = str(bit)
                if str_bit not in frame_offsets[str_offset]['bits']:
                    continue

                value = int.from_bytes(value_buf, 'big', signed=False)
                value = (value & (0x01 << bit)) > 0
                frame_entry = frame_offsets[str_offset]['bits'][str_bit]
                decoded_values.append([frame_entry, value])
        elif item_type == 'a':  # ascii HEX string
            value = ascii_hex_to_string(value_buf)
            decoded_values.append([frame_offsets[str_offset], value])

        return decoded_values, value_len

    def _decode_frame(self, data):
        """Decode one frame. Return all decoded values."""
        data_len = len(data)
        sender = format(data[0], '02x')
        receiver_raw = format(data[1], '02x')
        receiver = format(int(receiver_raw, 16) & 0x7f, '02x')
        poll = (receiver_raw != receiver)
        frametype = format(data[2], '02x')
        offset = int(data[3])
        start_offset = offset
        decoded_values = []

        while offset - start_offset < data_len - 2:
            if frametype not in self._decoding_table['frames']:
                return None

            frame = self._decoding_table['frames'][frametype]
            data_pos = offset - start_offset

            # check if current offset exists in the decoding table
            str_offset = str(offset)
            if str_offset not in frame['offsets']:
                offset += 1
                continue

            val_list, item_len = self._decode_value(frame, data[4:], data_pos)
            decoded_values.extend(val_list)

            offset += item_len

        for frame_entry, value in decoded_values:
            self._logger.debug(
                f'Decoded({frame["name"]}): {frame_entry}. [VALUE={value}]')

        return decoded_values

    def _read(self, raw=False, resp_type=Response.EMS):
        """Read a single frame and return the decoded values."""
        if not self._is_open:
            return None

        # return from read-queue if not empty
        self._read_queue.empty()
        if resp_type == Response.EMS:
            try:
                item = self._read_queue.get_nowait()
                self._logger.debug('Returning EMS response from READ_QUEUE')
                return item
            except queue.Empty:
                pass

        data_pos = 1
        done = False
        while not done:
            try:
                data_len = ord(self._port.read())
                data = self._port.read(data_len)
            except serial.serialutil.SerialException as e:
                self._logger.error(
                    f'Serial error during read (disconnected?): {e}')
                self.close()
                raise EmsError('Serial read error (disconnected?)')

            decoded_values = []

            self._logger.debug(f'RAW RECEIVED DATA: {data}')
            if data_len > 0 and data_len == len(data):
                if raw:
                    return data

                if data[0] == Response.EMS.value and data_len >= 8:
                    decoded_values = self._decode_frame(data[data_pos:])
                    if resp_type == Response.EMS:
                        return decoded_values
                    else:  # polling for gateway response => store EMS response to queue to be read later
                        self._read_queue.put(decoded_values)
                elif data[0] == Response.GATEWAY.value:
                    self._logger.debug(f'Gateway response: {data[data_pos:]}')
                    if resp_type == Response.GATEWAY:
                        return data[data_pos:]
                    else:
                        self._logger.warn(
                            'Received gateway response in regular EMS read.')
                else:
                    self._logger.warn('Unknown data received.')
            else:
                raise EmsError('Received data length mismatch.')

    def _send_cmd(self, cmd):
        """Write request to the serial port."""
        length = (len(cmd) + 1).to_bytes(1, byteorder='big')
        crc = calculate_crc(cmd).to_bytes(1, byteorder='big')
        msg = length + cmd + crc

        self._logger.debug('Writing to GW: {}'.format(binascii.hexlify(msg)))
        self._port.write(msg)

    def open(self):
        """Open the serial port. Throw SerialException if failed."""
        if not self._is_open:
            try:
                self._port.open()
                self._is_open = True
                self._logger.debug(f'Serial port {self._port.port} opened.')
            except serial.SerialException:
                self._logger.error(
                    f'Could not open serial port {self._port.port}')
                raise EmsError(f'Could not open serial port {self._port.port}')

    def close(self):
        """Close the serial port."""
        if self._is_open:
            self._port.close()
            self._is_open = False
            self._logger.debug(f'Serial port {self._port.port} closed.')

    def get_ems_command(self, name):
        """Get byte-represented EMS command from a given name."""
        try:
            cmd = self._config.get('commands', name)
            return bytearray.fromhex(cmd)
        except configparser.NoOptionError:
            self._logger.warn(f'Command {name} does not exist.')
            return None
        except configparser.NoSectionError:
            self._logger.error(
                'Could not find "commands" section in the config file.')
            return None

    def get_config(self):
        """Get the internal config object."""
        return self._config

    def read(self, raw=False):
        """Read one EMS frame and return decoded values."""
        return self._read(raw, Response.EMS)

    def write(self, cmd):
        """Write the given command to the EMS and wait for a gateway (Arduino) response."""
        self._send_cmd(cmd)  # send command

        # wait for gateway response
        resp = self._read(raw=False, resp_type=Response.GATEWAY)
        self._logger.debug(f'write: got GATEWAY response: {resp}')

        return resp[1], resp[2]  # return GatewayResponse, optional data
