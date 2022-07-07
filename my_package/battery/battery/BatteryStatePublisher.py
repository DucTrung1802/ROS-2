#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from battery.BatteryManager import BatteryManager

# Node parameters
PUBLISH_FREQUENCY = 0.05
NODE_NAME = "battery_state_publisher"


def checkConditions():
    global PUBLISH_PERIOD

    if PUBLISH_FREQUENCY <= 0:
        raise Exception("PUBLISH_FREQUENCY must be an positive integer!")
    PUBLISH_PERIOD = 1 / PUBLISH_FREQUENCY


class BatteryStatePublisher(Node):
    """
  Create a BatteryStatePublisher class, which is a subclass of the Node class.
  The class publishes the battery state of an object at a specific time interval.
  """

    def __init__(self, node_name, battery_manager):
        """
    Class constructor to set up the node
    """
        super().__init__(node_name)
        self.battery_manager = battery_manager

        self.battery_state_publisher = self.create_publisher(
            BatteryState, "/battery_status", 10
        )

        self.timer = self.create_timer(PUBLISH_PERIOD, self.publish)

        self.battery_voltage = self.battery_manager.getVoltage()
        self.percent_charge_level = self.battery_manager.getPercentChargeLevel()

    def publish(self):
        """
    Callback function.
    This function gets called at the specific time interval.
    We decrement the battery charge level to simulate a real-world battery.
    """
        msg = BatteryState()  # Create a message of this type
        msg.voltage = self.battery_voltage
        msg.percentage = self.percent_charge_level
        self.battery_state_publisher.publish(msg)  # Publish BatteryState message


def setup():
    checkConditions()


def loop(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    battery_manager = BatteryManager()

    # Create the node
    battery_state_pub = BatteryStatePublisher(NODE_NAME, battery_manager)

    # Spin the node so the callback function is called.
    # Publish any pending messages to the topics.
    rclpy.spin(battery_state_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    battery_state_pub.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


def main():
    setup()
    loop()


if __name__ == "__main__":
    main()
