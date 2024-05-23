import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (IncludeLaunchDescription, RegisterEventHandler,
                            TimerAction)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    package_name = "diffbot"
    package_share_directory = get_package_share_directory(package_name)
    slam_toolbox_file = os.path.join(
        package_share_directory, "launch/slam_bringup", "online_async_launch.py"
    )
    nav2_toolbox_file = os.path.join(
        package_share_directory, "launch/nav_bringup", "navigation_launch.py"
    )

    rviz_params = os.path.abspath(
        os.path.join(package_share_directory, "config", "nav2_default_view.rviz")
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_params],
        output={"both": "log"},
    )
    # Include SLAM launch file
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_file)
    )

    # Include Nav2 launch file with a delay
    delayed_nav2_toolbox = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(nav2_toolbox_file))
        ],
    )
    return LaunchDescription(
        [
            rviz,
            slam_toolbox,
            delayed_nav2_toolbox,
        ]
    )


if __name__ == "__main__":
    ld = generate_launch_description()
    print(ld)
