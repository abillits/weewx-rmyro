#!/usr/bin/env python
"""
rmyro - WeeWX driver for RM Young ResponseOne 92000 weather transmitters
Author: Andrew Billits

Derived from WX5x0 WeeWX driver by Mathew Wall:
https://github.com/matthewwall/weewx-wxt5x0

Notes:
========================================================
1) This driver is compatible with ResponseOne 92000 weather transmitters only. ResponseOne 91000 (Wind Only) weather transmitters are not compatible.

2) The data transmitted from ResponseOne weather transmitters does not include units and similar information. Therefore this drivers assumes the weather transmitter is configured as follows:

Pressure Unit: hPA
Temperature Unit: C
Wind Speed Unit: M/S
Serial Output Format: ASCII (*not ASCII Polled* - See #4 below)
Wind Output Format: Polar
Output Interval: 1000 (1000 seems to be safe - feel free to try lower or higher)

3) Only Serial ASCII modes (RS-232, RS-485 Half or Full) are supported by this driver. NMEA and SDI modes are not supported.

4) The ResponseOne weather transmitters are quite slow to respond to commands. During testing polled mode topped out at 1 data frame every 3.4 seconds. Auto-transmit mode achieved up to three data frames per second.

Fast data reception is important for ResponseOne weather transmitters since they do not capture/transmit wind gust data. Due to this limitation, this driver does not support polled mode. It expects a fast auto-transmitted stream of ASCII data from the unit.

5) Due to #4, this driver does not care what the address of the unit is. It will grab data from whatever unit is transmitting on the specified port.

6) This driver auto-detects whether the rain bucket has been enabled.

Installation:
========================================================
Place rmyro.py into weewx user directory (ex: /usr/share/weewx/user)
Modify your /etc/weewx/weewx.conf to use the rmyro driver by selecting:
station-type = rmyro under [Station]:

[Station]
    ...
    # Set to type of station hardware. There must be a corresponding stanza
    # in this file with a 'driver' parameter indicating the driver to be used.
    station_type = rmyro

Add a section in the weewx.conf file for the rmyro driver, it should look similar to this (but modified as needed):
[rmyro]
    model = RM Young ResponseOne
    port = /dev/ttyUSB0
    driver = user.rmyro

Useful Commands:
========================================================
/etc/init.d/weewx stop
/etc/init.d/weewx start
PYTHONPATH=/usr/share/weewx python /usr/share/weewx/user/rmyro.py --port /dev/ttyUSB0 --baud 38400
PYTHONPATH=/usr/share/weewx python /usr/share/weewx/user/rmyro.py --port /dev/ttyUSB_Primary --baud 38400
PYTHONPATH=/usr/share/weewx python /usr/share/weewx/user/rmyro.py --port /dev/ttyUSB_Primary --baud 38400 --enable-secondary true
ls /dev/tty*
"""

from __future__ import with_statement
import syslog
import time
import math
import subprocess
import weewx
import weewx.drivers
import weeutil.weeutil
from weewx.engine import StdService
import weewx.units
import weewx.wxformulas
import schemas.wview
import io

DRIVER_NAME = 'rmyro'
DRIVER_VERSION = '0.1'

MPS_PER_KPH = 0.277778
MPS_PER_MPH = 0.44704
MPS_PER_KNOT = 0.514444
MBAR_PER_PASCAL = 0.01
MBAR_PER_BAR = 1000.0
MBAR_PER_MMHG = 1.33322387415
MBAR_PER_INHG = 33.8639
MM_PER_INCH = 25.4
CM2_PER_IN2 = 6.4516


def loader(config_dict, _):
    return rmyro_driver(**config_dict[DRIVER_NAME])

def confeditor_loader():
    return rmyro_configuration_editor()


def logmsg(level, msg):
    syslog.syslog(level, 'rmyro: %s' % msg)

def logdbg(msg):
    logmsg(syslog.LOG_DEBUG, msg)

def loginf(msg):
    logmsg(syslog.LOG_INFO, msg)

def logerr(msg):
    logmsg(syslog.LOG_ERR, msg)

def map_sensor_data(x, in_min, in_max, out_min, out_max):
    return float((x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min)

def _fmt(x):
    return ' '.join(["%0.2X" % ord(c) for c in x])


class Station(object):
    def __init__(self, port, baud, secondary):
        self.terminator = '\r'
        self.primary_port = port
        self.primary_baudrate = baud
        self.secondary = secondary
        self.secondary_port = '/dev/ttyUSB_Secondary'
        self.secondary_baudrate = '38400'
        self.timeout = 3 # seconds
        self.primary_device = None
        self.secondary_device = None

    def open(self):
        import serial
        logdbg("open serial port %s" % self.primary_port)
        self.primary_device = serial.Serial(self.primary_port, self.primary_baudrate, timeout=self.timeout)
        self.primary_device_io = io.TextIOWrapper(io.BufferedRWPair(self.primary_device, self.primary_device, 1),newline = '\r',line_buffering = True)
        if "enabled" in self.secondary:
            self.secondary_device = serial.Serial(self.secondary_port, self.secondary_baudrate, timeout=self.timeout)
            self.secondary_device_io = io.TextIOWrapper(io.BufferedRWPair(self.secondary_device, self.secondary_device, 1),newline = '\r',line_buffering = True)

    def close(self):
        if self.primary_device is not None:
            logdbg("close serial port %s" % self.primary_port)
            self.primary_device.close()
            #self.primary_device_io.close()
            self.primary_device = None
        if self.secondary_device is not None:
            logdbg("close serial port %s" % self.secondary_port)
            self.secondary_device.close()
            #self.secondary_device_io.close()
            self.secondary_device = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, _, value, traceback):
        self.close()

    def send_primary_cmd(self, cmd):
        self.primary_device.write(cmd + self.terminator)

    def get_primary_data(self, cmd):
        #self.primary_device.flushInput()
        #self.send_primary_cmd(cmd)
        #line = self.primary_device.readline()
        self.primary_device.flushInput()
        self.primary_device_io.flush()
        line = self.primary_device_io.readline()
        #loginf("Debug 123")
        if line:
            line.replace('\x00', '') # eliminate any NULL characters
        return line

    def send_secondary_cmd(self, cmd):
        self.primary_device.write(cmd + self.terminator)

    def get_secondary_data(self, cmd):
        #self.secondary_device.flushInput()
        #self.send_secondary_cmd(cmd)
        #line = self.secondary_device.readline()
        self.secondary_device.flushInput()
        self.secondary_device_io.flush()
        line = self.secondary_device_io.readline()
        if line:
            line.replace('\x00', '') # eliminate any NULL characters
        return line

    def get_primary_sensor_data(self):
        return self.get_primary_data('')

    def get_secondary_sensor_data(self):
        return self.get_secondary_data('')

    #def bucket_counter_reset(self):
    #    self.send_cmd_primary('') #Maybe later

    @staticmethod
    def parse_primary_sensor_data(raw):
        #With Bucket
        #0 100.00 180.5 22.1 50.1 0998.6 00000 00*08
        #ADDRESS WIND_SPEED WIND_DIR TEMP HUM PRES BUCKET_TIPS STATUS*CRC

        #Without Bucket
        #0 100.00 180.5 22.1 50.1 0998.6 00*08
        #ADDRESS WIND_SPEED WIND_DIR TEMP HUM PRES STATUS*CRC

        parsed = dict()
        parts = raw.strip().split(' ')

        if len(parts) is 7 and '*' in parts[-1] and '.' in parts[-2] and len(parts[0]) is 1: #without rain bucket
            #parsed['address'] = float(parts[0])
            parsed['wind_speed'] = float(parts[1])
            parsed['wind_dir'] = float(parts[2])
            parsed['temperature'] = float(parts[3])
            parsed['humidity'] = float(parts[4])
            parsed['pressure'] = float(parts[5])

        if len(parts) is 8 and '*' in parts[-1] and '.' not in parts[-2] and len(parts[0]) is 1: #with rain bucket
            #parsed['address'] = float(parts[0])
            parsed['wind_speed'] = float(parts[1])
            parsed['wind_dir'] = float(parts[2])
            parsed['temperature'] = float(parts[3])
            parsed['humidity'] = float(parts[4])
            parsed['pressure'] = float(parts[5])
            parsed['rain'] = float(parts[6]) / 10 #each bucket tip is 0.1 mm

            #if float(parts[1]) > float('7'):
            #    loginf(raw)
            #    loginf(parsed['wind_speed'])

        return parsed

    @staticmethod
    def parse_secondary_sensor_data(raw):
        #9 v.v v.v v.v v.v 00*CC
        #ADDRESS SOIL_TEMP SOIL_MOISTURE UV_INDEX SOLAR_RADIATION STATUS*CRC

        parsed = dict()
        parts = raw.strip().split(' ')
        #if len(parts) > 5 and '*' in parts[-1]:
        if len(parts) is 6 and '*' in parts[-1] and len(parts[0]) is 1:
            #parsed['address'] = float(parts[0])
            parsed['soilTemp1'] = float((float(parts[1])*41.67) -40) #For "Vegetronix VH400 Soil Temperature Sensor": C = (V * 41.67) - 40 OR F = (V * 75.006) - 40
            #parsed['soilMoist1'] = float(map_sensor_data(float(parts[2]), 0, 3.0, 0, 100)) #For "Vegetronix VH400 Soil Moisture Sensor": 0-3V = 0-100%
            #loginf("Soil moisture: %s" % float(parts[2]))
            if float(parts[2]) <= 1.1: #For "Vegetronix VH400 Soil Moisture Sensor": https://www.vegetronix.com/Products/VH400/VH400-Piecewise-Curve.phtml
                parsed['soilMoist1'] = (10*float(parts[2]))-1
            if float(parts[2]) > 1.1 and float(parts[2]) <= 1.3:
                parsed['soilMoist1'] = (25*float(parts[2]))-17.5
            if float(parts[2]) > 1.3 and float(parts[2]) <= 1.82:
                parsed['soilMoist1'] = (48.08*float(parts[2]))-47.5
            if float(parts[2]) > 1.82 and float(parts[2]) <= 2.2:
                parsed['soilMoist1'] = (26.32*float(parts[2]))-7.89
            if float(parts[2]) > 2.2:
                parsed['soilMoist1'] = (62.5*float(parts[2]))-87.5
            if parsed['soilMoist1'] > 100: #max 100%
                parsed['soilMoist1'] = 100
            parsed['UV'] = round(float(map_sensor_data(float(parts[3]), 0, 2.0, 0, 10)), 1) #For "Spectrum UV Light Sensor": UMol/m2 s = V * 100 - Very rough mapping to UV Index
            parsed['radiation'] = (float(parts[4]) * 500) * 0.86 #For "Spectrum Silicon Pyranometer": W/m2 = V * 500 - 0.86 is a calibration factor
            if math.floor(parsed['radiation']) < 2:
                parsed['radiation'] = float('0.0')

        return parsed

class rmyro_configuration_editor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """
[rmyro]
    # This section is for RM Young ResponseOne weather transmitters
    model = ResponseOne

    # The port to which the weather transmitter is connected
    port = /dev/ttyUSB0

    # The driver to use
    driver = user.rmyro
"""

    def prompt_for_settings(self):
        print "Specify the model"
        model = self._prompt('model', 'ResponseOne')
        print "Specify the serial port on which the weather transmitter is connected, for"
        print "example /dev/ttyUSB0 or /dev/ttyS0."
        port = self._prompt('port', '/dev/ttyUSB0')
        return {'port': port}


class rmyro_driver(weewx.drivers.AbstractDevice):
    DEFAULT_PORT = '/dev/ttyUSB0'

    DEFAULT_BAUD = '38400'

    DEFAULT_SECONDARY = 'disabled'

    # map sensor names to schema names
    DEFAULT_MAP = {
        'windDir': 'wind_dir',
        'windSpeed': 'wind_speed',
        'outTemp': 'temperature',
        'outHumidity': 'humidity',
        'pressure': 'pressure',
        'rain_total': 'rain',
        }

    def __init__(self, **stn_dict):
        loginf('driver version is %s' % DRIVER_VERSION)
        self._model = stn_dict.get('model', 'ResponseOne')
        self._max_tries = int(stn_dict.get('max_tries', 5))
        self._retry_wait = int(stn_dict.get('retry_wait', 10))
        self._poll_interval = int(stn_dict.get('poll_interval', 1))
        self._sensor_map = dict(rmyro_driver.DEFAULT_MAP)
        baud = stn_dict.get('baud', rmyro_driver.DEFAULT_BAUD)
        baud = int(stn_dict.get('baud', baud))
        port = stn_dict.get('port', rmyro_driver.DEFAULT_PORT)
        secondary = stn_dict.get('secondary', rmyro_driver.DEFAULT_SECONDARY)
        secondary = stn_dict.get('secondary', secondary)
        self._secondary = secondary
        self.last_rain_total = None
        self._station = Station(port, baud, secondary)
        self._station.open()

    def closePort(self):
        self._station.close()

    @property
    def hardware_name(self):
        return self._model

    def genLoopPackets(self):
        while True:
            for cnt in range(self._max_tries):
                try:
                    raw_primary = self._station.get_primary_sensor_data()
                    logdbg("primary raw: %s" % _fmt(raw_primary))
                    data = Station.parse_primary_sensor_data(raw_primary)
                    logdbg("primary parsed: %s" % data)
                    #if self._secondary is 'enabled':
                    if "enabled" in self._secondary:
                        raw_secondary = self._station.get_secondary_sensor_data()
                        logdbg("primary raw: %s" % _fmt(raw_secondary))
                        data_secondary = Station.parse_secondary_sensor_data(raw_secondary)
                        logdbg("secondary parsed: %s" % data_secondary)
                        data.update(data_secondary)

                    packet = self._data_to_packet(data)
                    logdbg("mapped: %s" % packet)
                    if packet:
                        yield packet
                    break
                except IOError, e:
                    logerr("Failed attempt %d of %d to read data: %s" %
                           (cnt + 1, self._max_tries, e))
                    logdbg("Waiting %d seconds" % self._retry_wait)
                    time.sleep(self._retry_wait)
            else:
                raise weewx.RetriesExceeded("Read failed after %d tries" %
                                            self._max_tries)
            #Commenting this out by default - ResponseOne weather transmitters only read sensor data at a set interval or (in the case of this driver) when polled - thus polling needs to happen as quickly as possible to catch gusts
            #if self._poll_interval:
            #    time.sleep(self._poll_interval)

    def _data_to_packet(self, data):
        # if there is a mapping to a schema name, use it.  otherwise use the
        # sensor naming native to the hardware.
        packet = dict()
        for name in data:
            obs = name
            for field in self._sensor_map:
                if self._sensor_map[field] == name:
                    obs = field
                    break
            packet[obs] = data[name]
        if packet:
            packet['dateTime'] = int(time.time() + 0.5)
            packet['usUnits'] = weewx.METRICWX
        if 'rain_total' in packet:
            packet['rain'] = self._delta_rain(
                packet['rain_total'], self.last_rain_total)
            self.last_rain_total = packet['rain_total']
        return packet

    @staticmethod
    def _delta_rain(rain, last_rain):
        if last_rain is None:
            loginf("skipping rain measurement of %s: no last rain" % rain)
            return None
        if rain < last_rain:
            loginf("rain counter wraparound detected: new=%s last=%s" %
                   (rain, last_rain))
            return rain
        return rain - last_rain


# define a main entry point for basic testing of the weather transmitter without weewx
# engine and service overhead.  invoke this as follows from the weewx root dir:
#
# PYTHONPATH=/usr/share/weewx python /usr/share/weewx/user/rmyro.py --port /dev/ttyUSB0 --baud 38400 --debug

if __name__ == '__main__':
    import optparse
    secondary_enabled = 'disabled'
    usage = """%prog [options] [--debug] [--help]"""
    syslog.openlog('rmyro', syslog.LOG_PID | syslog.LOG_CONS)
    syslog.setlogmask(syslog.LOG_UPTO(syslog.LOG_INFO))
    parser = optparse.OptionParser(usage=usage)
    parser.add_option('--version', action='store_true',
                      help='display driver version')
    parser.add_option('--debug', action='store_true',
                      help='display diagnostic information while running')
    parser.add_option('--port',
                      help='serial port to which the weather transmitter is connected',
                      default=rmyro_driver.DEFAULT_PORT)
    parser.add_option('--baud', type=int,
                      help='baud rate', default=38400)
    parser.add_option('--enable-secondary',
                      help='enables secondary data')
    (options, args) = parser.parse_args()

    if options.version:
        print "%s driver version %s" % (DRIVER_NAME, DRIVER_VERSION)
        exit(1)

    if options.debug:
        syslog.setlogmask(syslog.LOG_UPTO(syslog.LOG_DEBUG))

    if options.enable_secondary:
        secondary_enabled = 'enabled'

    with Station(options.port, options.baud, secondary_enabled) as s:
        if secondary_enabled is 'enabled':
            while True:
                primary_data = s.get_primary_sensor_data().strip()
                print int(time.time()), primary_data
                primary_parsed = Station.parse_primary_sensor_data(primary_data)
                print primary_parsed
                secondary_data = s.get_secondary_sensor_data().strip()
                print int(time.time()), secondary_data
                secondary_parsed = Station.parse_secondary_sensor_data(secondary_data)
                print secondary_parsed
                print '----------------------------------'
        else:
            while True:
                primary_data = s.get_primary_sensor_data().strip()
                print int(time.time()), primary_data
                primary_parsed = Station.parse_primary_sensor_data(primary_data)
                print primary_parsed
