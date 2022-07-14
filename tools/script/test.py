#! /usr/bin/env python

import serial

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)

ser.isOpen()
ser.write("12345")
ser.inWaiting()
ser.read(4)
