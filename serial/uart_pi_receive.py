#!/usr/bin/python3
 
import time
import serial
import re
 
ser = serial.Serial(
	port = '/dev/ttyUSB0',
	baudrate = 115200,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS,
	timeout = 1,
	xonxoff=0,
	rtscts=0
)

ser.setDTR(False)
time.sleep(0.1)
ser.flushInput()
ser.setDTR(True)

print("Raspberry's receiving: ")

try:
	while True:
		s = ser.readline()		
		data = s.decode("windows-1252")		# decode s
		data = data.rstrip()			# cut "\r\n" at last of string
		clean_data = re.sub("[^A-Za-z0-9]","",data)
		print(clean_data)				# print string
 
except KeyboardInterrupt:
	ser.close()
