from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        Node(
            package = 'rclcpp_mqtt_client',
            name = 'ros_connection_bridge',
            executable = 'ros_connection_bridge',
            output = 'screen'
        ),
        Node(
            package = 'rclcpp_mqtt_client',
            name = 'ros_mqtt_bridge',
            executable = 'ros_mqtt_bridge',
            output = 'screen'
        )
    ])