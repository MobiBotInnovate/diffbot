import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
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
        os.path.join(package_share_directory, "launch", "rsp.launch.py")
    )
    online_async_launch_file = os.path.abspath(
        os.path.join(package_share_directory, "launch", "online_async_launch.py")
    )
    navigation_launch_file = os.path.abspath(
        os.path.join(package_share_directory, "launch", "navigation_launch.py")
    )
    twist_mux_params_file = os.path.abspath(
        os.path.join(package_share_directory, "config", "twist_mux.yaml")
    )

    # Print statements for debugging
    print("Gazebo params file:", gazebo_params_file)
    print("RViz params file:", rviz_params)
    print("World file:", world_file)
    print("RSP launch file:", rsp_launch_file)
    print("Online async launch file:", online_async_launch_file)
    print("Navigation launch file:", navigation_launch_file)

    # Verify that the files exist
    if not os.path.isfile(rsp_launch_file):
        raise FileNotFoundError(f"RSP launch file not found: {rsp_launch_file}")
    if not os.path.isfile(online_async_launch_file):
        raise FileNotFoundError(
            f"Online async launch file not found: {online_async_launch_file}"
        )
    if not os.path.isfile(navigation_launch_file):
        raise FileNotFoundError(
            f"Navigation launch file not found: {navigation_launch_file}"
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
        arguments=["-topic", "robot_description", "-entity", "diffbot"],
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

    online_async = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(online_async_launch_file)
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_file),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            rsp,
            gazebo,
            spawn_entity,
            diff_drive_spawner,
            joint_broad_spawner,
            twix_mux,
            # online_async,
            rviz,
            # Uncomment this if you want to include the navigation launch file
            # navigation,
        ]
    )


if __name__ == "__main__":
    ld = generate_launch_description()
    print(ld)
