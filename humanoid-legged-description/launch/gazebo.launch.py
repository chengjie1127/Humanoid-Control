from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    description_share = get_package_share_directory('humanoid_legged_description')

    empty_world_launch = os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
    model_file = os.path.join(description_share, 'urdf', 'humanoid_legged_origin.urdf')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(empty_world_launch)
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_model',
            arguments=['-entity', 'humanoid_legged_description', '-file', model_file],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/calibrated', 'std_msgs/msg/Bool', '{data: true}'],
            output='screen'
        ),
    ])
