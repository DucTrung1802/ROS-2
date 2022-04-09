from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_pubsub',
            namespace='ns1',
            executable='my_publisher',
            name='my_publisher_node'
        ),
        Node(
            package='py_pubsub',
            namespace='ns1',
            executable='my_subscriber',
            name='my_subscriber_node'
        )
    ])
