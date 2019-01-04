import serial
import configparser
import json
import logging
from enum import Enum
from .util import ascii_hex_to_string


class Response(Enum):
    EMS = 0xF4
    GATEWAY = 0xA0

class EmsError(Exception):
    pass

class Ems:
    def __init__(self,
                 config_filename='pyems/assets/default.cfg',
                 decoding_table_filename='pyems/assets/decoding_table.json',
                 open_port=True):
        self._port = serial.Serial()
        self._is_open = False

        self._load_config(config_filename)
        self._load_decoding_table(decoding_table_filename)

        if open_port:
            self.open()

    def __del__(self):
        self.close()

    def _create_logger(self, log_level, log_format, date_format):
        self._logger = logging.getLogger('pyems_logger')
        self._logger.setLevel(log_level)

        handler = logging.StreamHandler()
        handler.setLevel(log_level)

        formatter = logging.Formatter(fmt=log_format, datefmt=date_format)
        handler.setFormatter(formatter)

        self._logger.addHandler(handler)

    def _load_config(self, filename):
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
        try:
            with open(filename, encoding='utf-8') as f:
                self._decoding_table = json.load(f)
            self._logger.debug('Decoding table loaded.')
        except Exception as e:
            self._logger.critical('Failed to load the decoding table.')
            raise e

    def _decode_value(self, frame, data, offset):
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
            if 'unsigned' in frame_offsets[str_offset]:
                unsigned = frame_offsets[str_offset]['unsigned']
            else:
                unsigned = False

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

            val_list, item_len = self._decode_value(frame, data, data_pos)
            decoded_values.extend(val_list)

            offset += item_len

        for frame_entry, value in decoded_values:
            self._logger.debug(f'Decoded: {frame_entry}. Value: {value}')

        return decoded_values

    def open(self):
        """Open the serial port. Throw SerialException if failed."""
        if not self._is_open:
            try:
                self._port.open()
                self._is_open = True
                self._logger.debug(f'Serial port {self._port.port} opened.')
            except serial.SerialException:
                self._logger.error(f'Could not open serial port {self._port.port}')
                raise EmsError(f'Could not open serial port {self._port.port}')

    def close(self):
        if self._is_open:
            self._port.close()
            self._is_open = False
            self._logger.debug(f'Serial port {self._port.port} closed.')

    def read(self, raw=False, datapoints_only=False):
        if not self._is_open:
            return None

        data_pos = 1
        data_len = ord(self._port.read())
        data = self._port.read(data_len)
        decoded_values = []

        if data_len > 0 and data_len == len(data):
            if raw:
                return data

            if data[0] == Response.EMS.value and data_len >= 8:
                decoded_values = self._decode_frame(data[data_pos:])
            else:
                self._logger.warn('Unknown data received.')

            return decoded_values
        else:
            raise EmsError('Received data length mismatch.')

    def write(self):
        pass