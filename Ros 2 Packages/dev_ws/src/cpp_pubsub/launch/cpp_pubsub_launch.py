from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
       Node(
         package='cpp_pubsub',
         namespace='ns1', # Make sure this matches the subscriber's namespace
         executable='my_publisher', # Name of the executable
         name='minimal_publisher' # Any name is fine
       ),
       Node(
         package='cpp_pubsub',
         namespace='ns1', # Make sure this matches the publisher's namespace
         executable='my_subscriber', # Name of the executable
         name='minimal_subscriber' # Any name is fine
       )
    ])
