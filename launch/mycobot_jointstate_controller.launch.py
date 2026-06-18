from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('command_topic', default_value='/mycobot/joint_commands'),
        DeclareLaunchArgument('state_topic', default_value='/mycobot/joint_states'),
        DeclareLaunchArgument('ee_pose_topic', default_value='/mycobot/ee_pose'),
        DeclareLaunchArgument('position_unit', default_value='rad'),
        DeclareLaunchArgument('command_speed', default_value='30'),
        DeclareLaunchArgument('command_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('state_publish_rate_hz', default_value='10.0'),
        Node(
            package='mycobot_jointstate_controller',
            executable='jointstate_controller_node',
            name='mycobot_jointstate_controller',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
                'command_topic': LaunchConfiguration('command_topic'),
                'state_topic': LaunchConfiguration('state_topic'),
                'ee_pose_topic': LaunchConfiguration('ee_pose_topic'),
                'position_unit': LaunchConfiguration('position_unit'),
                'command_speed': LaunchConfiguration('command_speed'),
                'command_rate_hz': LaunchConfiguration('command_rate_hz'),
                'state_publish_rate_hz': LaunchConfiguration('state_publish_rate_hz'),
            }],
        ),
    ])
