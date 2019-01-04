import logging
from .pyems import Ems, EmsError

ems = None
try:
    ems = Ems()

    data = ems.read() # read one frame
    for value_meta, value in data:
        print(f'{value_meta}\n    VALUE = {value}')

except EmsError as e:
    logging.error(e)
finally:
    if ems is not None:
        ems.close()
