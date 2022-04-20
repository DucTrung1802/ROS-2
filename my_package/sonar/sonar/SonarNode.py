# Libraries
import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time
import Sonar

# Node parameters
PUBLISH_FREQUENCY = 100
NODE_NAME = "sonar"

timer1 = time.time()


def checkConditions():
    global PUBLISH_PERIOD

    if PUBLISH_FREQUENCY <= 0:
        raise Exception("PUBLISH_FREQUENCY must be an positive integer!")
    PUBLISH_PERIOD = 1 / PUBLISH_FREQUENCY


class SonarNode(Node):
    def __init__(self, node_name, sonar_instance):
        super().__init__(node_name)
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

def setup():
    checkConditions()

def loop():
    global timer1
    rclpy.init()
    sonar = Sonar(trigger_pin, echo_pin, min_range, max_range, field_of_view)
    sonar_node = SonarNode(NODE_NAME, sonar)

    try:
        while True:
            if time.time() - timer1 >= PUBLISH_PERIOD:
                rclpy.spin_once(sonar)
                timer1 = time.time()

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()


def main():
    setup()
    loop()


if __name__ == "__main__":
    main()
