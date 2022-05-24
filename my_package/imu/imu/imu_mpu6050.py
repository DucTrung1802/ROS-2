import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import RPi.GPIO as GPIO

import time
from mpu6050 import mpu6050

# Node parameters
NODE_NAME = "imu_mpu6050"
PUBLISH_FREQUENCY = 100

timer1 = 0

# Node parameters
PUBLISH_PERIOD = 0


def checkConditions():
    global PUBLISH_PERIOD

    if PUBLISH_FREQUENCY <= 0:
        raise Exception("PUBLISH_FREQUENCY must be an positive integer!")
    PUBLISH_PERIOD = 1 / PUBLISH_FREQUENCY


class IMUPublisher(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 1)

        timer_period1 = 0  # seconds
        self.timer1 = self.create_timer(timer_period1, self.timer_callback)

    def timer_callback(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.linear_acceleration.x = accel_data["x"]
        msg.linear_acceleration.y = accel_data["y"]
        msg.linear_acceleration.z = accel_data["z"]
        msg.angular_velocity.x = gyro_data["x"]
        msg.angular_velocity.y = gyro_data["y"]
        msg.angular_velocity.z = gyro_data["z"]
        self.imu_pub.publish(msg)


def getIMUData():
    global accel_data, gyro_data
    try:
        mpu = mpu6050(0x68)
        accel_data = mpu.get_accel_data()
        gyro_data = mpu.get_gyro_data()
    except:
        print("An error has occurred while getting data from IMU MPU 6050!")


def setup():
    checkConditions()


def loop(args=None):
    global POS, timer1
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()

    try:
        while True:
            if time.time() - timer1 >= PUBLISH_PERIOD:
                getIMUData()
                rclpy.spin_once(imu_publisher)
                timer1 = time.time()

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
