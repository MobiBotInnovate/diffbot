import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "diffbot"
    package_share_directory = get_package_share_directory(package_name)

    gazebo_params_file = os.path.abspath(
        os.path.join(package_share_directory, "config", "gazebo_params.yaml")
    )
    rviz_params = os.path.abspath(
        os.path.join(package_share_directory, "config", "nav2_default_view.rviz")
    )
    world_file = os.path.abspath(
        os.path.join(package_share_directory, "worlds", "turtlebot3_world.world")
    )
    rsp_launch_file = os.path.abspath(
        os.path.join(package_share_directory, "launch/robot_bringup", "rsp.launch.py")
    )
    twist_mux_params_file = os.path.abspath(
        os.path.join(package_share_directory, "config", "twist_mux.yaml")
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch_file),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.abspath(
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py",
                )
            )
        ),
        launch_arguments={
            "world": world_file,
            "extra_gazebo_args": "--ros-args --params-file " + gazebo_params_file,
            "verbose": "false",
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "diffbot",
            "-x",
            "-2.0",  # Set the x position here
            "-y",
            "0.0",
            "-z",
            "0.0",
        ],
        output="screen",
    )
    diff_drive_spawner = Node(
        package="controller_manager", executable="spawner", arguments=["diff_cont"]
    )
    joint_broad_spawner = Node(
        package="controller_manager", executable="spawner", arguments=["joint_broad"]
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
        parameters=[twist_mux_params_file, {"use_sim_time": True}],
        remappings=[("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")],
    )

    return LaunchDescription(
        [
            rsp,
            gazebo,
            spawn_entity,
            diff_drive_spawner,
            joint_broad_spawner,
            twix_mux,
            rviz,
        ]
    )


if __name__ == "__main__":
    ld = generate_launch_description()
    print(ld)
