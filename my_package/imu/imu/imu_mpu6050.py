# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

from logging import exception
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import tf_transformations
import threading
import RPi.GPIO as GPIO

import time
import board
import adafruit_mpu6050

# Initial pose parameters
ROLL = 0.0
PITCH = 0.0
YAW = 0.0

# Node parameters
NODE_NAME = "imu_mpu6050"
PUBLISH_FREQUENCY = 100

# Node parameters
PUBLISH_PERIOD = 0


class Calibrator(object):
    def __init__(self):
        self.__mpu = adafruit_mpu6050.MPU6050(board.I2C())
        self.__cali_accel_data = [0.0, 0.0, 0.0]
        self.__cali_gyro_data = [0.0, 0.0, 0.0]

    def calibrate(self, number_of_run=1):
        if int(number_of_run) and number_of_run > 0:
            for i in range(number_of_run):
                self.__cali_accel_data[0] += self.__mpu.acceleration[0]
                self.__cali_accel_data[1] += self.__mpu.acceleration[1]
                self.__cali_accel_data[2] += self.__mpu.acceleration[2]

                self.__cali_gyro_data[0] += self.__mpu.gyro[0]
                self.__cali_gyro_data[1] += self.__mpu.gyro[1]
                self.__cali_gyro_data[2] += self.__mpu.gyro[2]

                time.sleep(0.01)

            self.__cali_accel_data[0] /= float(number_of_run)
            self.__cali_accel_data[1] /= float(number_of_run)
            self.__cali_accel_data[2] /= float(number_of_run)

            self.__cali_gyro_data[0] /= float(number_of_run)
            self.__cali_gyro_data[1] /= float(number_of_run)
            self.__cali_gyro_data[2] /= float(number_of_run)

        else:
            raise exception("Invalid number of run!")

    def getCalibratedAccelData(self):
        return self.__cali_accel_data

    def getCalibratedGyroData(self):
        return self.__cali_gyro_data


CALIBRATOR = Calibrator()


def checkConditions():
    global PUBLISH_PERIOD

    if PUBLISH_FREQUENCY <= 0:
        raise Exception("PUBLISH_FREQUENCY must be an positive integer!")
    PUBLISH_PERIOD = 1 / PUBLISH_FREQUENCY


def setupInitialPose(roll, pitch, yaw):
    global quaternion
    quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
    return quaternion


class IMUPublisher(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.imu_pub = self.create_publisher(Imu, "/imu/data_raw", 10)

        self.timer1 = self.create_timer(PUBLISH_PERIOD, self.timer_callback)

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        initial_pose_quaternion = setupInitialPose(ROLL, PITCH, YAW)

        # need test x,y,z,w or w,x,y,z
        # msg.orientation.x = initial_pose_quaternion[0]
        # msg.orientation.y = initial_pose_quaternion[1]
        # msg.orientation.z = initial_pose_quaternion[2]
        # msg.orientation.w = initial_pose_quaternion[3]

        msg.linear_acceleration.x = (
            accel_data[0] - CALIBRATOR.getCalibratedAccelData()[0]
        )
        msg.linear_acceleration.y = (
            accel_data[1] - CALIBRATOR.getCalibratedAccelData()[1]
        )
        msg.linear_acceleration.z = accel_data[2]

        msg.angular_velocity.x = gyro_data[0] - CALIBRATOR.getCalibratedGyroData()[0]
        msg.angular_velocity.y = gyro_data[1] - CALIBRATOR.getCalibratedGyroData()[1]
        msg.angular_velocity.z = gyro_data[2] - CALIBRATOR.getCalibratedGyroData()[2]

        for i in range(9):
            if i == 0:
                msg.orientation_covariance[i] = 1000000000000.0
                msg.angular_velocity_covariance[i] = 1000000000000.0
                msg.linear_acceleration_covariance[i] = 1.0e-5
            elif i == 4:
                msg.orientation_covariance[i] = 1000000000000.0
                msg.angular_velocity_covariance[i] = 1000000000000.0
                msg.linear_acceleration_covariance[i] = 1.0e-5
            elif i == 8:
                msg.orientation_covariance[i] = 0.001
                msg.angular_velocity_covariance[i] = 0.001
                msg.linear_acceleration_covariance[i] = 1000000000000.0
            else:
                msg.orientation_covariance[i] = 0.0
                msg.angular_velocity_covariance[i] = 0.0
                msg.linear_acceleration_covariance[i] = 0.0

        self.imu_pub.publish(msg)


def getIMUData(mpu):
    global accel_data, gyro_data
    try:
        accel_data = mpu.acceleration
        gyro_data = mpu.gyro
        # print(".")

    except:
        print("An error has occurred while getting data from IMU MPU 6050!")

    # finally:
    #     print("accel data: " + str(accel_data))
    #     print("gyro data: " + str(gyro_data))
    #     print()


def task_1():
    global flag_1
    i2c = board.I2C()  # uses board.SCL and board.SDA
    mpu = adafruit_mpu6050.MPU6050(i2c)
    while True:

        if flag_1:
            break

        getIMUData(mpu)
        time.sleep(0.01)


def task_2():
    global flag_2
    rclpy.init()
    imu_publisher = IMUPublisher()
    time.sleep(0.1)
    while True:

        if flag_2:
            break

        rclpy.spin(imu_publisher)


def threadingHandler():
    global flag_1, flag_2, flag_3

    flag_1 = False
    flag_2 = False
    flag_3 = False

    thread_1 = threading.Thread(target=task_1)
    thread_2 = threading.Thread(target=task_2)
    # thread_3 = threading.Thread(target=task_3)

    thread_1.start()
    thread_2.start()
    # thread_3.start()

    thread_1.join()
    thread_2.join()
    # thread_3.join()


def calibrate():
    CALIBRATOR.calibrate(100)


def setup():
    checkConditions()
    calibrate()


def loop(args=None):
    try:
        threadingHandler()

    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()
    # rclpy.shutdown()


def main():
    setup()
    loop()


if __name__ == "__main__":
    main()
