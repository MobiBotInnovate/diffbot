import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_name = "diffbot"
    package_share_directory = get_package_share_directory(package_name)
    rsp_launch_file = os.path.abspath(
        os.path.join(package_share_directory, "launch", "rsp.launch.py")
    )
    rplidar_launch_file = os.path.join(
        package_share_directory, "launch", "rplidar.launch.py"
    )

    rsp = IncludeLaunchDescription(PythonLaunchDescriptionSource(rsp_launch_file))

    diff_drive_spawner = Node(
        package="controller_manager", executable="spawner", arguments=["diff_cont"]
    )
    joint_broad_spawner = Node(
        package="controller_manager", executable="spawner", arguments=["joint_broad"]
    )
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_file),
        launch_arguments={"use_sim_time": "false"}.items(),
    )

    return LaunchDescription(
        [
            rsp,
            diff_drive_spawner,
            joint_broad_spawner,
            rplidar,
        ]
    )
