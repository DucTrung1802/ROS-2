# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import adafruit_mpu6050

i2c = board.I2C()  # uses board.SCL and board.SDA
mpu = adafruit_mpu6050.MPU6050(i2c)

mpu_acc_x = 0.0
mpu_acc_y = 0.0
mpu_acc_z = 0.0

mpu_gyro_x = 0.0
mpu_gyro_y = 0.0
mpu_gyro_z = 0.0

amount_of_data = int(input("Amount of data: "))
for i in range(amount_of_data):
    mpu_acc_x += mpu.acceleration[0]
    mpu_acc_y += mpu.acceleration[1]
    mpu_acc_z += mpu.acceleration[2]

    mpu_gyro_x += mpu.gyro[0]
    mpu_gyro_y += mpu.gyro[1]
    mpu_gyro_z += mpu.gyro[2]

    time.sleep(0.01)
    # print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (mpu.acceleration))
    # print("Gyro X:%.2f, Y: %.2f, Z: %.2f rad/s" % (mpu.gyro))
    # print("Temperature: %.2f C" % mpu.temperature)

print(
    "Acceleration: X = "
    + str(mpu_acc_x / amount_of_data)
    + " Y = "
    + str(mpu_acc_y / amount_of_data)
    + " Z = "
    + str(mpu_acc_z / amount_of_data)
)

print()

print(
    "Gyro: X = "
    + str(mpu_gyro_x / amount_of_data)
    + " Y = "
    + str(mpu_gyro_y / amount_of_data)
    + " Z = "
    + str(mpu_gyro_z / amount_of_data)
)
