import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory("diffbot"))
    xacro_file = os.path.join(pkg_path, "description", "robot_ros2_control.urdf.xacro")
    robot_description_config = Command(
        [
            "xacro ",
            xacro_file,
            " sim_mode:=",
            use_sim_time,
        ]
    )
    # Create a robot_state_publisher node
    params = {
        "robot_description": robot_description_config,
        "use_sim_time": use_sim_time,
    }
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params],
    )

    # Launch!
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use sim time if true",
            ),
            node_robot_state_publisher,
        ]
    )
