import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    package_name = "diffbot"
    package_share_directory = get_package_share_directory(package_name)

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory(package_name),
                    "launch/robot_bringup",
                    "rsp.launch.py",
                )
            ]
        ),
        launch_arguments={"use_sim_time": "false", "use_ros2_control": "true"}.items(),
    )
    twist_mux_params = os.path.join(
        get_package_share_directory(package_name), "config", "twist_mux.yaml"
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
    )
    controller_params = os.path.join(
        get_package_share_directory("diffbot"), "config", "ros2_controller.yaml"
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params],
    )
    # delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        parameters=[controller_params],
        output="screen",
    )

    #    delayed_diff_drive_spawner = RegisterEventHandler(
    #        event_handler=OnProcessStart(
    #            target_action=controller_manager,
    #            on_start=[diff_drive_spawner],
    #        )
    #    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        parameters=[controller_params],
        output="screen",
    )

    #    delayed_joint_broad_spawner = RegisterEventHandler(
    #        event_handler=OnProcessStart(
    #            target_action=controller_manager,
    #            on_start=[joint_broad_spawner],
    #        )
    #    )

    return LaunchDescription(
        [
            # rsp,
            # delayed_controller_manager,
            # delayed_diff_drive_spawner,
            # delayed_joint_broad_spawner,
            diff_drive_spawner,
            joint_broad_spawner,
            twist_mux,
        ]
    )


if __name__ == "__main__":
    ld = generate_launch_description()
    print(ld)
