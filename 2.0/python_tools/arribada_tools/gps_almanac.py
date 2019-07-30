#!/usr/bin/python2.7

import logging
from arribada_tools import gps_config, backend, interface

def apply_almanac(serial, baud, comms_backend, almanac_file):
    gps_backend = None
    if serial:
        gps_backend = gps_config.GPSSerialBackend(serial, baudrate=baud)
    else:
        gps_backend = gps_config.GPSBridgedBackend(comms_backend)
        interface.ConfigInterface(comms_backend).gps_config(True)

    gps_backend.read(1024)
    mga_ano_data = almanac_file.read()
    cfg = gps_config.GPSConfig(gps_backend)
    cfg.mga_ano_session(mga_ano_data)
