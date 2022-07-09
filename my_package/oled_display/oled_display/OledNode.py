import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


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
        self.subscription = self.create_subscription(
            Float64, "topic", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.__oled.clear()
        self.__oled.add_text(
            text=str(f"{round(msg.data)}%"), horizontal_align="right", vertical_align=0
        )
        self.__oled.display()

    def clear_display(self):
        self.__oled.clear()


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
