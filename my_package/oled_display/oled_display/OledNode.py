from ast import arg
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import UInt8
from std_msgs.msg import String

from PIL import ImageFont


from oled_display.Oled import Oled

# Node parameters
PUBLISH_FREQUENCY = 10
NODE_NAME = "oled_node"
OLED_FREQUENCY = 60

FONT_SET_DICT = {
    "order_number": {
        "font": ImageFont.truetype("DejaVuSerif-Bold.ttf", 45),
        "width": 32,
        "height": 44,
    },
    "battery": {"font": ImageFont.load_default(), "width": 6, "height": 10},
    "starting": {
        "font": ImageFont.truetype("DejaVuSerif-Bold.ttf", 20),
        "width": 14.625,
        "height": 15,
    },
    "waiting": {
        "font": ImageFont.truetype("DejaVuSerif-Bold.ttf", 20),
        "width": 13,
        "height": 15,
    },
}


def checkConditions():
    global PUBLISH_PERIOD, OLED_PERIOD

    if PUBLISH_FREQUENCY <= 0:
        raise Exception("PUBLISH_FREQUENCY must be an positive integer!")
    PUBLISH_PERIOD = 1 / PUBLISH_FREQUENCY

    if OLED_FREQUENCY <= 0:
        raise Exception("OLED_FREQUENCY must be an positive integer!")
    OLED_PERIOD = 1 / OLED_FREQUENCY


class OledNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.__node_name = node_name
        self.__oled = Oled()

        self.__initializeBatterySubscription()

        self.__initializeLowBatterySubscription()

        self.__initializeSetGoalSubscription()

        self.__display_cycle_timer = self.create_timer(0, self.__displayOled)

        self.__oled.clear()

        self.__oled.add_text(
            text=str("STARTING"),
            font_set=FONT_SET_DICT["starting"],
            horizontal_align="center",
            vertical_align=22.5,
        )
        self.__oled.display()
        time.sleep(2)

    def __initializeBatterySubscription(self):
        self.__battery_data = BatteryState()
        self.__battery_subscription = self.create_subscription(
            BatteryState, "/battery_state", self.__batteryCallback, 10
        )
        self.__battery_subscription  # prevent unused variable warning

    def __initializeLowBatterySubscription(self):
        self.__low_battery_alert = int(0)
        self.__low_battery_subscription = self.create_subscription(
            UInt8, "/low_battery", self.__lowBatteryCallback, 10
        )
        self.__low_battery_subscription  # prevent unused variable warning

    def __initializeSetGoalSubscription(self):
        self.__current_goal = String()
        self.__current_goal_subscription = self.create_subscription(
            String, "/goal_progress", self.__currentGoalCallback, 10
        )
        self.__current_goal_subscription  # prevent unused variable warning

    def __lowBatteryCallback(self, msg):
        self.__low_battery_alert = msg.data

    def __batteryCallback(self, msg):
        self.__battery_data = msg

    def __currentGoalCallback(self, msg):
        self.__current_goal = msg

    def __displayOled(self):
        self.__oled.clear()

        self.__oled.add_text(
            text=str(f"{round(self.__battery_data.percentage)}%"),
            font_set=FONT_SET_DICT["battery"],
            horizontal_align="right",
            vertical_align=0,
        )

        if self.__current_goal.data == "" or self.__current_goal.data == "None":
            self.__oled.add_text(
                text=str("WAITING.."),
                font_set=FONT_SET_DICT["waiting"],
                horizontal_align="center",
                vertical_align=22.5,
            )

        elif self.__current_goal.data != "":
            self.__oled.add_text(
                text=str(f"{round(int(self.__current_goal.data))}"),
                font_set=FONT_SET_DICT["order_number"],
                horizontal_align="center",
                vertical_align=10,
            )

        if self.__low_battery_alert == 1:
            self.__oled.add_text(
                text=str("LOW BATTERY!"),
                font_set=FONT_SET_DICT["battery"],
                horizontal_align="left",
                vertical_align=0,
            )

        elif self.__low_battery_alert == 0:
            self.__oled.add_text(
                text=str(""),
                font_set=FONT_SET_DICT["battery"],
                horizontal_align="left",
                vertical_align=0,
            )

        self.__oled.display()

    def clear_display(self):
        self.__oled.shutdown()


def setup():
    checkConditions()


def loop():
    rclpy.init()
    oled_publisher = OledNode(NODE_NAME)
    try:
        rclpy.spin(oled_publisher)

    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Oled has stopped by User")
        oled_publisher.clear_display()
        exit(0)

    # oled.add_text(text="", horizontal_align="left", vertical_align=15)
    # oled.add_text(text="", horizontal_align="center", vertical_align=25)
    # oled.add_text(text="hello", horizontal_align="right", vertical_align=35)


def main():
    setup()
    loop()


if __name__ == "__main__":
    main()
