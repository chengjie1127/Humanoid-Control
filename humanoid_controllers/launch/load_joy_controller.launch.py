from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    use_gnome_terminal = os.path.exists('/usr/bin/gnome-terminal.real') or os.path.exists('/usr/bin/gnome-terminal')
    return LaunchDescription([
        DeclareLaunchArgument('taskFile', default_value=PathJoinSubstitution([FindPackageShare('humanoid_interface'), 'config/mpc/task.info'])),
        DeclareLaunchArgument('referenceFile', default_value=PathJoinSubstitution([FindPackageShare('humanoid_interface'), 'config/command/reference.info'])),
        DeclareLaunchArgument('urdfFile', default_value=PathJoinSubstitution([FindPackageShare('humanoid_legged_description'), 'urdf/humanoid_legged_control.urdf'])),
        DeclareLaunchArgument('urdfFileOrigin', default_value=PathJoinSubstitution([FindPackageShare('humanoid_legged_description'), 'urdf/humanoid_legged_origin.urdf'])),
        DeclareLaunchArgument('gaitCommandFile', default_value=PathJoinSubstitution([FindPackageShare('humanoid_interface'), 'config/command/gait.info'])),
        
        Node(
            package='humanoid_dummy',
            executable='humanoid_gait_command',
            name='humanoid_gait_command',
            output='screen',
            parameters=[{
                'taskFile': LaunchConfiguration('taskFile'),
                'referenceFile': LaunchConfiguration('referenceFile'),
                'urdfFile': LaunchConfiguration('urdfFile'),
                'urdfFileOrigin': LaunchConfiguration('urdfFileOrigin'),
                'gaitCommandFile': LaunchConfiguration('gaitCommandFile')
            }],
            **({'prefix': 'gnome-terminal --'} if use_gnome_terminal else {})
        ),
        
        Node(
            package='humanoid_controllers',
            executable='humanoid_target_trajectories_publisher',
            name='humanoid_target',
            output='screen',
            parameters=[{
                'taskFile': LaunchConfiguration('taskFile'),
                'referenceFile': LaunchConfiguration('referenceFile'),
                'urdfFile': LaunchConfiguration('urdfFile'),
                'urdfFileOrigin': LaunchConfiguration('urdfFileOrigin'),
                'gaitCommandFile': LaunchConfiguration('gaitCommandFile')
            }]
        ),
        
        Node(
            package='humanoid_controllers',
            executable='joy_controller_node',
            name='joy_controller',
            output='screen',
            parameters=[{
                'taskFile': LaunchConfiguration('taskFile'),
                'referenceFile': LaunchConfiguration('referenceFile'),
                'urdfFile': LaunchConfiguration('urdfFile'),
                'urdfFileOrigin': LaunchConfiguration('urdfFileOrigin'),
                'gaitCommandFile': LaunchConfiguration('gaitCommandFile')
            }]
        ),
        
        Node(
            package='mujoco_sim',
            executable='humanoid_sim.py',
            name='humanoid_sim',
            output='screen'
        ),
        
        Node(
            package='mujoco_sim',
            executable='joy.py',
            name='teleop_joy',
            output='screen'
        ),
        
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        )
    ])
