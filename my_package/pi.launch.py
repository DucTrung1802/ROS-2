from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="imu_complementary_filter",
                executable="complementary_filter_node",
                name="complementary_filter_gain_node",
                output="screen",
                parameters=[
                    {"do_bias_estimation": True},
                    {"do_adaptive_gain": True},
                    {"use_mag": False},
                    {"gain_acc": 0.01},
                    {"gain_mag": 0.01},
                ],
            ),
            Node(
                package="rplidar_ros2",
                executable="rplidar_scan_publisher",
                name="rplidar_scan_publisher",
                output="screen",
                parameters=[
                    {"serial_port": "/dev/ttyUSB0"},
                    {"serial_baudrate": 115200},
                    {"frame_id": "lidar_link"},
                    {"inverted": False},
                    {"angle_compensate": True},
                ],
            ),
            Node(
                package="imu",
                executable="imu",
                name="imu",
                output="screen",
                parameters=[],
            ),
            Node(
                package="esp32",
                executable="esp32",
                name="esp32",
                output="screen",
                parameters=[],
            ),
            Node(
                package="sonar",
                executable="sonar",
                name="sonar",
                output="screen",
                parameters=[],
            ),
            Node(
                package="oled_display",
                executable="oled",
                name="oled",
                output="screen",
                parameters=[],
            ),
            Node(
                package="ir",
                executable="ir_sensor",
                name="ir_sensor",
                output="screen",
                parameters=[],
            ),
        ]
    )

