import os

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
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

    #motor_serial_driver_file = os.path.abspath(os.path.join(package_share_directory, "launch", "motor_driver.launch")
    #)

    rsp = IncludeLaunchDescription(PythonLaunchDescriptionSource(rsp_launch_file))
    #mrd = IncludeLaunchDescription(PythonLaunchDescriptionSource(motor_serial_driver_file)

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params = os.path.join(
            get_package_share_directory('diffbot'),  
            'config',
            'ros2_controller.yaml'
            )

    controller_manager = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description},
                controller_params],
            )
    # Directly below the controller manager node
    delayed_controller_manager = TimerAction(period=3.0,actions=[controller_manager])

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

    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch_file),
        launch_arguments={"use_sim_time": "false"}.items(),
    )

    return LaunchDescription(
        [
            rsp,
            delayed_controller_manager,
            delayed_diff_drive_spawner,
            delayed_joint_broad_spawner,
            rplidar,
        ]
    )
