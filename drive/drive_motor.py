#!/usr/bin/python3

import subprocess
import time
from click import prompt
import serial
import re

SKIP_SERIAL_LINES = 12
LIDAR_USB_NAME = "FTDI USB Serial Device"
MCU_USB_NAME = "cp210x"
BAUD_RATE = 115200
FREQUENCY = 1
RECEIVING_PERIOD = 1

timer = time.time();
MCUSerialObject = None
foundMCU = False
foundLidar = False
serialData = ""

def checkFrequency():
    global FREQUENCY
    if (FREQUENCY <= 0):
        FREQUENCY = 1
    RECEIVING_PERIOD = 1/ FREQUENCY

def getMCUSerial():
    global foundMCU, foundLidar
    MCUSerial = None
    output = subprocess.getstatusoutput("dmesg | grep ttyUSB")
    stringDevices = list(output[1].split("\n"))
    stringDevices.reverse()

    for device in stringDevices:
        if device.find(MCU_USB_NAME) > 0 and not foundMCU:
            if device.find("disconnected") > 0:
                raise Exception("MCU is disconnected!")
            else:
                # print("MCU in serial: " + device.split()[-1])
                foundMCU = True
                MCUSerial = device.split()[-1]
                break

        # elif (device.find(LIDAR_USB_NAME) > 0 and not foundLidar):
        #     if (device.find("disconnected") > 0):
        #         raise Exception("Lidar is disconnected!")
        #     else:
        #         # print("Lidar in serial: " + device.split()[-1])
        #         foundLidar = True
        #         continue

    return MCUSerial


def initializeSerial():
    global MCUSerialObject
    MCUSerial = getMCUSerial()
    if MCUSerial:
        MCUSerialObject = serial.Serial(
            port="/dev/" + MCUSerial,
            baudrate=BAUD_RATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1,
            write_timeout=1,
        )

    skipLines = SKIP_SERIAL_LINES
    MCUSerialObject.setDTR(False)
    time.sleep(0.5)
    MCUSerialObject.reset_input_buffer()
    MCUSerialObject.setDTR(True)

    while skipLines:
        # Skip some lines of serial when MCU is reseted
        MCUSerialObject.readline()
        skipLines = skipLines - 1


def readSerialData():
    global serialData
    rawData = MCUSerialObject.readline()
    serialData = rawData.decode("windows-1252")  # decode s
    serialData = serialData.rstrip()  # cut "\r\n" at last of string
    serialData = re.sub("[^A-Za-z0-9\s]", "", serialData)  # filter regular characters
    # print(data)  # print string


def manuallyWrite():
    # A command is appended with "#" to mark as finish
    command = prompt("Command") + "#"
    command = bytes(command, "utf-8")
    MCUSerialObject.write(command)



def setup():
    checkFrequency();
    initializeSerial()


def loop():
    try:
        while True:
            manuallyWrite()
            if (time.time() - timer >= RECEIVING_PERIOD):
                readSerialData()
                print(serialData)
                timer = time.time()

    except KeyboardInterrupt:
        MCUSerialObject.close()


def main():
    setup()
    loop()


if __name__ == "__main__":
    main()
