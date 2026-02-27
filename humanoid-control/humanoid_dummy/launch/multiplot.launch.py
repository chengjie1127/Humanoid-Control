from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    metrics_config = LaunchConfiguration('metrics_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'metrics_config',
            default_value=PathJoinSubstitution([FindPackageShare('humanoid_interface'), 'config', 'multiplot', 'zero_velocity.xml'])
        ),
        Node(
            package='rqt_multiplot',
            executable='rqt_multiplot',
            name='mpc_metrics',
            arguments=['--multiplot-run-all', '--multiplot-config', metrics_config],
            output='screen',
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('ocs2_ros_interfaces'), 'launch', 'performance_indices.launch.py'])
            ),
            launch_arguments={'mpc_policy_topic_name': 'humanoid_mpc_policy'}.items(),
        ),
    ])
