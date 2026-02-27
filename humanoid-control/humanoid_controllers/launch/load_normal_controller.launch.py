from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz = LaunchConfiguration('rviz')
    task_file = LaunchConfiguration('taskFile')
    reference_file = LaunchConfiguration('referenceFile')
    urdf_file = LaunchConfiguration('urdfFile')
    urdf_file_origin = LaunchConfiguration('urdfFileOrigin')
    gait_command_file = LaunchConfiguration('gaitCommandFile')

    shared_params = {
        'taskFile': task_file,
        'referenceFile': reference_file,
        'urdfFile': urdf_file,
        'urdfFileOrigin': urdf_file_origin,
        'gaitCommandFile': gait_command_file,
    }

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'taskFile',
            default_value=PathJoinSubstitution([FindPackageShare('humanoid_interface'), 'config', 'mpc', 'task.info'])
        ),
        DeclareLaunchArgument(
            'referenceFile',
            default_value=PathJoinSubstitution([FindPackageShare('humanoid_interface'), 'config', 'command', 'reference.info'])
        ),
        DeclareLaunchArgument(
            'urdfFile',
            default_value=PathJoinSubstitution([FindPackageShare('humanoid_legged_description'), 'urdf', 'humanoid_legged_control.urdf'])
        ),
        DeclareLaunchArgument(
            'urdfFileOrigin',
            default_value=PathJoinSubstitution([FindPackageShare('humanoid_legged_description'), 'urdf', 'humanoid_legged_origin.urdf'])
        ),
        DeclareLaunchArgument(
            'gaitCommandFile',
            default_value=PathJoinSubstitution([FindPackageShare('humanoid_interface'), 'config', 'command', 'gait.info'])
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('humanoid_dummy'), 'rviz', 'humanoid.rviz'])],
            output='screen',
            condition=IfCondition(rviz),
        ),
        Node(
            package='humanoid_dummy',
            executable='humanoid_gait_command',
            name='humanoid_gait_command',
            output='screen',
            parameters=[shared_params],
        ),
        Node(
            package='humanoid_controllers',
            executable='humanoid_target_trajectories_publisher',
            name='humanoid_target',
            output='screen',
            parameters=[shared_params],
        ),
        Node(
            package='humanoid_controllers',
            executable='normal_controller_node',
            name='normal_controller',
            output='screen',
            parameters=[shared_params],
        ),
        Node(
            package='mujoco_sim',
            executable='humanoid_sim.py',
            name='humanoid_sim',
            output='screen',
        ),
        Node(
            package='mujoco_sim',
            executable='teleop.py',
            name='teleop',
            output='screen',
        ),
    ])
