import subprocess

foundMCU = False
foundLidar = False

output = subprocess.getstatusoutput("dmesg | grep ttyUSB")
stringDevices = list(output[1].split("\n"))
stringDevices.reverse()

for device in stringDevices:
    if (device.find("ch341-uart") > 0 and not foundMCU):
        if (device.find("disconnected") > 0):
            raise Exception("MCU is disconnected!")
        else:
            print("MCU in serial: " + device.split()[-1])
            foundMCU = True
            continue

    elif (device.find("FTDI USB Serial Device") > 0 and not foundLidar):
        if (device.find("disconnected") > 0):
            raise Exception("Lidar is disconnected!")
        else:
            print("Lidar in serial: " + device.split()[-1])
            foundLidar = True
            continue
