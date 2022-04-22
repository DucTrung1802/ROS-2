# Libraries
import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time
from sonar.SonarClass import Sonar

# Node parameters
PUBLISH_FREQUENCY = 10
NODE_NAME = "sonar"

timer1 = time.time()


def checkConditions():
    global PUBLISH_PERIOD

    # GPIO Mode (BOARD / BCM)
    GPIO.setmode(GPIO.BCM)

    if PUBLISH_FREQUENCY <= 0:
        raise Exception("PUBLISH_FREQUENCY must be an positive integer!")
    PUBLISH_PERIOD = 1 / PUBLISH_FREQUENCY


class SonarNode(Node):
    def __init__(self, node_name, sonar_array_instance):
        if isinstance(sonar_array_instance, list) and len(sonar_array_instance) < 0:
            raise Exception("There is no Sonar data!")

        super().__init__(node_name)
        self.array_sonar = sonar_array_instance
        self.array_publisher = []
        for i in range(len(sonar_array_instance)):
            self.array_publisher.append(
                self.create_publisher(Range, "sonar_" + str(i + 1), 1)
            )
        self.timer = self.create_timer(0, self.timer_callback)

    def timer_callback(self):
        for i in range(len(self.array_sonar)):
            msg = Range()
            msg.header.frame_id = "/sonar_frame_" + str(i + 1)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.radiation_type = self.array_sonar[i].getRadiationType()
            msg.field_of_view = self.array_sonar[
                i
            ].getFieldOfView()  # rad ~ 45 degree (according to feature of HC-SR 04)
            msg.min_range = self.array_sonar[i].getMinRange()
            msg.max_range = self.array_sonar[i].getMaxRange()
            msg.range = self.array_sonar[i].getMeasureDistance()
            self.array_publisher[i].publish(msg)


def setup():
    checkConditions()


def loop():
    global timer1
    rclpy.init()
    sonar_array = []

    # Configure this when add more sonar sensors
    sonar_1 = Sonar(
        trigger_pin=17, echo_pin=27, min_range=0.04, max_range=1.5, field_of_view=0.78
    )
    sonar_2 = Sonar(
        trigger_pin=10, echo_pin=9, min_range=0.04, max_range=1.5, field_of_view=0.78
    )
    sonar_3 = Sonar(
        trigger_pin=5, echo_pin=6, min_range=0.04, max_range=1.5, field_of_view=0.78
    )
    sonar_4 = Sonar(
        trigger_pin=13, echo_pin=19, min_range=0.04, max_range=1.5, field_of_view=0.78
    )
    sonar_5 = Sonar(
        trigger_pin=23, echo_pin=24, min_range=0.04, max_range=1.5, field_of_view=0.78
    )

    sonar_array.append(sonar_1)
    sonar_array.append(sonar_2)
    sonar_array.append(sonar_3)
    sonar_array.append(sonar_4)
    sonar_array.append(sonar_5)

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
