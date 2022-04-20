# Libraries
import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time

timer1 = time.time()


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("sonar_publisher")
        self.publisher_1 = self.create_publisher(Range, "sonar", 1)
        self.timer1 = self.create_timer(0, self.timer_callback1)

    def timer_callback1(self):
        msg = Range()
        msg.header.frame_id = "/base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.radiation_type = 0
        msg.field_of_view = 0.78  # rad ~ 45 degree (according to feature of HC-SR 04)
        msg.min_range = 0.04
        msg.max_range = 2.0
        msg.range = distance()
        self.publisher_1.publish(msg)


def main(args=None):
    global timer1
    rclpy.init(args=args)
    sonar = MinimalPublisher()

    try:
        while True:
            if time.time() - timer1 >= 0.05:
                rclpy.spin_once(sonar)
                timer1 = time.time()

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
