#!/usr/bin/python3

import time
from click import prompt
import serial
import re

ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1,
    write_timeout=1
)

ser.setDTR(False)
time.sleep(0.5)
ser.reset_input_buffer()
ser.setDTR(True)
skip_serial = 5

print("Raspberry's sending: ")

try:
    while True:
        while (skip_serial):
            s = ser.readline()
            skip_serial = skip_serial - 1
            
        # A command is appended with "#" to mark as finish
        command = prompt("Command") + "#"
        # start = time.time()
        command = bytes(command, "utf-8")
        ser.write(command)
        # end = time.time()
        # print(end - start)
        s = ser.readline()		
        data = s.decode("windows-1252")		# decode s
        data = data.rstrip()			# cut "\r\n" at last of string
        clean_data = re.sub("[^A-Za-z0-9\s]","",data)
        print(clean_data)				# print string

except KeyboardInterrupt:
    ser.close()
