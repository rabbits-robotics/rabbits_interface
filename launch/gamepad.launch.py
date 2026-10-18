from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('max_omni_vel', default_value='0.4'))  # m/s
    ld.add_action(DeclareLaunchArgument('max_omni_rot_vel', default_value='1.0'))  # rad/s
    ld.add_action(DeclareLaunchArgument('max_yaw_rot_vel', default_value='2.0'))  # rad/s
    ld.add_action(DeclareLaunchArgument('max_pitch_rot_vel', default_value='2.0'))  # rad/s

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_linux_node',
        output='screen',
        parameters=[{
            'autorepeat_rate': 100.0,  # Hz（デフォルト: 20.0）
        }],
    )

    gamepad_node = Node(
        package='rabbits_interface',
        executable='gamepad_node',
        name='gamepad_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'max_omni_vel': LaunchConfiguration('max_omni_vel'),
            'max_omni_rot_vel': LaunchConfiguration('max_omni_rot_vel'),
            'max_yaw_rot_vel': LaunchConfiguration('max_yaw_rot_vel'),
            'max_pitch_rot_vel': LaunchConfiguration('max_pitch_rot_vel'),
        }],
    )

    ld.add_action(joy_node)
    ld.add_action(gamepad_node)

    return ld
