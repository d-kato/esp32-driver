# The ESP32 WiFi driver for Mbed OS
The Mbed OS driver for the ESP32 WiFi module.

## Firmware version
How to write mbed-os compatible firmware :
https://github.com/d-kato/GR-Boards_ESP32_Serial_Bridge

## Restrictions
- Setting up an UDP server is not possible
- The serial port does not have hardware flow control enabled. The AT command set does not either have a way to limit the download rate. Therefore, downloading anything larger than the serial port input buffer is unreliable. An application should be able to read fast enough to stay ahead of the network. This affects mostly the TCP protocol where data would be lost with no notification. On UDP, this would lead to only packet losses which the higher layer protocol should recover from.
