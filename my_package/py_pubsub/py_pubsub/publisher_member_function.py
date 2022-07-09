import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String
from std_msgs.msg import Float64

POS = 0
timer1 = time.time()
timer2 = time.time()


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_1 = self.create_publisher(Float64, "topic", 10)

        timer_period1 = 1  # seconds
        # bind = lambda x: self.timer_callback(x)
        self.timer1 = self.create_timer(timer_period1, self.timer_callback)

        self.i = 0.0

    def timer_callback(self):
        msg = Float64()
        msg.data = self.i
        self.publisher_1.publish(msg)
        self.get_logger().info("Publishing: " + str(self.i))
        self.i += 1


def main(args=None):
    global POS, timer1, timer2
    rclpy.init(args=args)
    minimal_publisher_1 = MinimalPublisher()
    # minimal_publisher_2 = MinimalPublisher()
    rclpy.spin(minimal_publisher_1)

    # while True:
    # rclpy.spin_once(minimal_publisher_1)
    # if time.time() - timer1 >= 0.0001:
    #     print(POS)
    #     POS += 1
    #     timer1 = time.time()

    # if time.time() - timer2 >= 0.001:
    #     rclpy.spin_once(minimal_publisher_1)
    #     timer2 = time.time()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
