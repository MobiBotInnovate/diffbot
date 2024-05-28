import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

set_env_vars_resources = (
    AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(get_package_share_directory("diffbot"), "worlds"),
    ),
)


def generate_launch_description():
    package_name = "diffbot"
    package_share_directory = get_package_share_directory(package_name)
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gazebo_params_file = os.path.abspath(
        os.path.join(package_share_directory, "config", "gazebo_params.yaml")
    )
    rviz_params = os.path.abspath(
        os.path.join(package_share_directory, "config", "nav2_default_view.rviz")
    )
    world = os.path.abspath(
        os.path.join(package_share_directory, "worlds", "turtlebot3_world.world")
    )
    rsp_launch_file = os.path.abspath(
        os.path.join(package_share_directory, "launch/robot_bringup", "rsp.launch.py")
    )
    twist_mux_params_file = os.path.abspath(
        os.path.join(package_share_directory, "config", "twist_mux.yaml")
    )
    robot_localization_file_path = os.path.join(
        package_share_directory, "config", "ekf.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_ros2_control = LaunchConfiguration("use_ros2_control", default="true")

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch_file),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_ros2_control": use_ros2_control,
        }.items(),
    )

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.abspath(
                os.path.join(
                    ros_gz_sim,
                    "launch",
                    "gz_sim.launch.py",
                )
            )
        ),
        launch_arguments={
            "gz_args": ["-r -s -v4 ", world],
            "on_exit_shutdown": "true",
            # "verbose": "false",
        }.items(),
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-g -v4 "}.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
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
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        condition=IfCondition(use_ros2_control),
        output="screen",
    )
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        condition=IfCondition(use_ros2_control),
        output="screen",
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
    # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[robot_localization_file_path],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use simulation time"
            ),
            DeclareLaunchArgument(
                "use_ros2_control", default_value="true", description="Use ROS2 control"
            ),
            rsp,
            gz_server,
            gzclient_cmd,
            spawn_entity,
            diff_drive_spawner,
            joint_broad_spawner,
            twix_mux,
            rviz,
            start_robot_localization_cmd,
        ]
    )


if __name__ == "__main__":
    ld = generate_launch_description()
    print(ld)
