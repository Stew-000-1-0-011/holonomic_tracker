"""tracker_node を起動する。sim:=true で模擬ロボットと円の目標も一緒に立てる。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = LaunchConfiguration('params')
    sim = LaunchConfiguration('sim')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params',
            default_value=PathJoinSubstitution(
                [FindPackageShare('omni3_tracker'), 'config', 'tracker_node.yaml']),
            description='tracker_node のパラメータファイル',
        ),
        DeclareLaunchArgument(
            'sim', default_value='false',
            description='模擬ロボット (fake_omni_robot) と円の目標を一緒に起動する',
        ),
        Node(
            package='omni3_tracker',
            executable='tracker_node',
            name='tracker_node',
            parameters=[params],
            output='screen',
        ),
        Node(
            package='omni3_tracker',
            executable='fake_omni_robot',
            name='fake_omni_robot',
            output='screen',
            condition=IfCondition(sim),
        ),
        Node(
            package='omni3_tracker',
            executable='circle_reference_publisher',
            name='circle_reference_publisher',
            remappings=[('reference', '/tracker_node/reference')],
            output='screen',
            condition=IfCondition(sim),
        ),
    ])
