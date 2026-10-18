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
    ld.add_action(DeclareLaunchArgument('fire_heat', default_value='10.0'))  # 1射の熱量増加量

    game_client_node = Node(
        package='rabbits_interface',
        executable='game_client_node',
        name='game_client_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'max_omni_vel': LaunchConfiguration('max_omni_vel'),
            'max_omni_rot_vel': LaunchConfiguration('max_omni_rot_vel'),
            'max_yaw_rot_vel': LaunchConfiguration('max_yaw_rot_vel'),
            'max_pitch_rot_vel': LaunchConfiguration('max_pitch_rot_vel'),
            'fire_heat': LaunchConfiguration('fire_heat'),
        }],
    )

    ld.add_action(game_client_node)

    return ld
