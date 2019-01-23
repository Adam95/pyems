import pytest

from pyems import Ems, EmsError


@pytest.fixture(scope='session')
def ems():
    # create Ems instance with a mock serial port
    ems_inst = Ems(test_mode=True)

    # put some data on the mock serial port
    with open('pyems/assets/EMS_controller_bin.log', 'rb') as f:
        ems_inst._port.write(f.read(2000))

    return ems_inst

def test_read(ems):
    data = ems.read()  # read test bytes from the mock serial port
    assert len(data) == 18  # read 18 values (UBAMonitorFast)

    # check some values if they are correctly decoded
    for value_meta, value in data:
        if value_meta['name'] == 'T_kotel_vyst_pozad':
            # integer value
            assert value == 35
        elif value_meta['name'] == 'T_boiler_stred':
            # float value
            assert value == 45.2
        elif value_meta['name'] == 'S_oc':
            assert value == True
        elif value_meta['name'] == 'T_zpatecka':
            # this value is missing
            assert value is None

def test_get_ems_command(ems):
    # test valid command defined in the default.cfg
    assert ems.get_ems_command('UBAMonitorSlow') is not None
    assert ems.get_ems_command('WorkingModeHC1') is not None
    assert ems.get_ems_command('ParameterHotWater') is not None
    assert ems.get_ems_command('UBAMonitorHotWater') is not None

    # test non-existing command
    assert ems.get_ems_command('Non-existing command') is None

def test_invalid_config():
    try:
        Ems(test_mode=True, config_filename='foobar')
        assert False
    except FileNotFoundError:
        assert True

def test_invalid_decoding_table():
    try:
        Ems(test_mode=True, decoding_table_filename='foobar')
        assert False
    except FileNotFoundError:
        assert True

