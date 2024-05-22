import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    package_name = "diffbot"
    package_share_directory = get_package_share_directory(package_name)

    rviz_params = os.path.abspath(
        os.path.join(package_share_directory, "config", "nav2_default_view.rviz")
    )
    twist_mux_params_file = os.path.abspath(
        os.path.join(package_share_directory, "config", "twist_mux.yaml")
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_params],
        output={"both": "log"},
    )

    twix_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params_file],
        remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
    )

    return LaunchDescription(
        [
            twix_mux,
            rviz,
        ]
    )


if __name__ == "__main__":
    ld = generate_launch_description()
    print(ld)
