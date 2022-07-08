# Import necessary modules
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import RPi.GPIO as GPIO
import time


HIGH = True
LOW = False

GPIO.setmode(GPIO.BCM)

NODE_NAME = "ir_node"

IR_PIN = 4

# Set up GPIO pins

# pull_up_down=GPIO.PUD_DOWN or pull_up_down=GPIO.PUD_UP
GPIO.setup(IR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


class IRPublisher(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.ir_publisher = self.create_publisher(Empty, "/input_at_waypoint/input", 1)
        self.timer = self.create_timer(0, self.publish)

    def publish(self):
        msg = Empty()
        self.ir_publisher.publish(msg)


def setup():
    # setup somthing (do 1 time)
    pass


def loop():
    # continuously do something

    rclpy.init()

    ir_node = IRPublisher(NODE_NAME)

    try:
        while True:
            # Read button state
            if not GPIO.input(IR_PIN):
                rclpy.spin_once(ir_node)
                print("okay, go")
                time.sleep(0.1)
            else:
                pass

            # wait 1 ms to give CPU chance to do other things
            time.sleep(0.001)

    except KeyboardInterrupt:  # if Ctrl C is pressed...
        print("Program stopped and furnace shut off.")  # print a clean exit message
    GPIO.cleanup()


def main():
    setup()
    loop()


if __name__ == "__main__":
    main()
