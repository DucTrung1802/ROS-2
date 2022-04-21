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
    def __init__(self, node_name, sonar_array_instance):
        if isinstance(sonar_array_instance, list) and len(sonar_array_instance) < 0:
            raise Exception("There is no Sonar data!")

        super().__init__(node_name)
        self.array_publisher = []
        for i in range(len(sonar_array_instance)):
            self.array_publisher.append(
                self.create_publisher(Range, "sonar_" + str(i + 1), 1)
            )
        self.timer = self.create_timer(0, self.timer_callback)

    def timer_callback(self):
        for i in range(len(self.array_publisher)):
            msg = Range()
            msg.header.frame_id = "/base_link"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.radiation_type = 0
            msg.field_of_view = (
                0.78
            )  # rad ~ 45 degree (according to feature of HC-SR 04)
            msg.min_range = 0.04
            msg.max_range = 2.0
            msg.range = i + 1
            self.array_publisher[i].publish(msg)


def setup():
    checkConditions()


def loop():
    global timer1
    rclpy.init()
    sonar_array = []
    sonar_1 = Sonar(
        trigger_pin=23, echo_pin=24, min_range=0.04, max_range=1.5, field_of_view=0.78
    )
    sonar_array.append(sonar_1)

    sonar_node = SonarNode(NODE_NAME, sonar_array)

    try:
        while True:
            if time.time() - timer1 >= PUBLISH_PERIOD:
                rclpy.spin_once(sonar_node)
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
