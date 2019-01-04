def calculate_crc(buffer):
    crc = 0
    for byte in buffer:
        d = 0
        if crc & 0x80:
            crc ^= 12
            d = 1
        crc = (crc << 1) & 0xfe
        crc |= d
        crc ^= byte
    return crc


def ascii_hex_to_string(ascii_hex):
    hex_result = ''
    text_result = ''
    for b in ascii_hex:
        if 33 <= b <= 126:
            text_result += chr(b)
        else:
            text_result += '?'
        hex_result += "%0.2X" % b
    return f'{hex_result} ({text_result})'
