from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_motor_demo',
            executable='driver',
            name='motor_driver',
            parameters=[{
                'serial_port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
                'baud_rate': 57600,
                'loop_rate': 30,
                'encoder_cpr': 2497
            }],
            output='screen'
        ),
    ])

