import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, RegisterEventHandler,
                            TimerAction, DeclareLaunchArgument)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    package_name = "diffbot"
    package_share_directory = get_package_share_directory(package_name)

    # Node to publish the robot description from the parameter
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(package_share_directory, 'description', 'robot.urdf.xacro')]), "use_sim_time": use_sim_time}],
    )

    controller_params = os.path.join(
        get_package_share_directory("diffbot"), "config", "ros2_controller.yaml"
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params],
    )
    delayed_controller_manager = TimerAction(period=5.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager", executable="spawner", arguments=["diff_cont"]
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager", executable="spawner", arguments=["joint_broad"]
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="use sim time if true",
                ),
            robot_state_publisher,
            delayed_controller_manager,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner,
        ]
    )
 
if __name__ == "__main__":
    ld = generate_launch_description()
    print(ld)   
