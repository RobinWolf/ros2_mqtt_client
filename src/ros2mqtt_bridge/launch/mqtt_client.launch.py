from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare




def generate_launch_description():
    mqtt_package = "ros2mqtt_bridge"

    mqtt_configs = PathJoinSubstitution([FindPackageShare(mqtt_package), "config", "mqtt_publisher_client.yaml"])
    mqtt_client_node = Node(
        package="ros2mqtt_bridge",
        executable="mqtt_client_publisher_node",
        parameters=[mqtt_configs]
    )

    nodes_to_start = [
        mqtt_client_node,
    ]

    return LaunchDescription(nodes_to_start) 