from ast import arg
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Empty
from std_msgs.msg import String


from oled_display.Oled import Oled

# Node parameters
PUBLISH_FREQUENCY = 10
NODE_NAME = "sonars"


def checkConditions():
    global PUBLISH_PERIOD

    if PUBLISH_FREQUENCY <= 0:
        raise Exception("PUBLISH_FREQUENCY must be an positive integer!")
    PUBLISH_PERIOD = 1 / PUBLISH_FREQUENCY


class OledNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.__node_name = node_name
        self.__oled = Oled()

        self.__initializeBatterySubscription()

        self.__initializeLowBatterySubscription()

        self.__initializeSetGoalSubscription()

        self.__display_cycle_timer = self.create_timer(0.03, self.__displayOled)

    def __initializeBatterySubscription(self):
        self.__battery_data = BatteryState()
        self.__battery_subscription = self.create_subscription(
            BatteryState, "/battery_state", self.__batteryCallback, 10
        )
        self.__battery_subscription  # prevent unused variable warning

    def __initializeLowBatterySubscription(self):
        self.__low_battery_alert = False
        self.__low_battery_subscription = self.create_subscription(
            Empty, "/low_battery", self.__lowBatteryCallback
        )
        self.__low_battery_subscription  # prevent unused variable warning

    def __initializeSetGoalSubscription(self):
        self.__current_goal = ""
        self.__current_goal_subscription = self.create_subscription(
            String, "/goal_progress", self.__currentGoalCallback
        )
        self.__current_goal_subscription  # prevent unused variable warning

    def __lowBatteryCallback(self, msg):
        self.__low_battery_alert = True

    def __batteryCallback(self, msg):
        self.__battery_data = msg

    def __currentGoalCallback(self, msg):
        self.__current_goal = msg

    def __displayOled(self):
        self.__oled.clear()

        self.__oled.add_text(
            text=str(f"{round(self.__battery_data.percentage)}%"),
            horizontal_align="right",
            vertical_align=0,
        )

        self.__oled.add_text(
            text=str(f"==> TABLE {self.__current_goal.data}"),
            horizontal_align="center",
            vertical_align=25,
        )

        if self.__low_battery_alert:
            self.__oled.add_text(
                text=str("LOW BATTERY!"), horizontal_align="center", vertical_align=40
            )

        self.__low_battery_alert = False

        self.__oled.display()


def setup():
    pass


def loop():
    rclpy.init()
    oled_publisher = OledNode(NODE_NAME)
    try:
        rclpy.spin(oled_publisher)

    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Oled has stopped by User")
        oled_publisher.clear_display()

    # oled.add_text(text="", horizontal_align="left", vertical_align=15)
    # oled.add_text(text="", horizontal_align="center", vertical_align=25)
    # oled.add_text(text="hello", horizontal_align="right", vertical_align=35)


def main():
    setup()
    loop()


if __name__ == "__main__":
    main()
