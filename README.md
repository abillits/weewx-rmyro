
#rmyro - WeeWX driver for RM Young ResponseOne 92000 weather transmitters
Author: Andrew Billits

Derived from WX5x0 WeeWX driver by Mathew Wall:
https://github.com/matthewwall/weewx-wxt5x0

#Notes:
#========================================================
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

#Installation:
#========================================================
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
