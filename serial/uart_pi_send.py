#!/usr/bin/python3

import time
from click import prompt
import serial

ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1,
    write_timeout=1
)

ser.setDTR(False)
time.sleep(0.1)
ser.reset_input_buffer()
ser.setDTR(True)

print("Raspberry's sending: ")

try:
    while True:
        # Finish a command with "#"
        command = prompt("Command")
        # start = time.time()
        command = bytes(command, "utf-8")
        ser.write(command)
        # end = time.time()
        # print(end - start)

except KeyboardInterrupt:
    ser.close()
