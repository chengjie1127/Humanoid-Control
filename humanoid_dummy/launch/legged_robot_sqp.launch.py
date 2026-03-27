from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('multiplot', default_value='false'),
        DeclareLaunchArgument('taskFile', default_value=PathJoinSubstitution([FindPackageShare('humanoid_interface'), 'config/mpc/task.info'])),
        DeclareLaunchArgument('referenceFile', default_value=PathJoinSubstitution([FindPackageShare('humanoid_interface'), 'config/command/reference.info'])),
        DeclareLaunchArgument('urdfFile', default_value=PathJoinSubstitution([FindPackageShare('humanoid_legged_description'), 'urdf/humanoid_legged_control.urdf'])),
        DeclareLaunchArgument('urdfFileOrigin', default_value=PathJoinSubstitution([FindPackageShare('humanoid_legged_description'), 'urdf/humanoid_legged_origin.urdf'])),
        DeclareLaunchArgument('gaitCommandFile', default_value=PathJoinSubstitution([FindPackageShare('humanoid_interface'), 'config/command/gait.info'])),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command(['cat ', LaunchConfiguration('urdfFileOrigin')]), value_type=str)
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('humanoid_dummy'), 'rviz/humanoid.rviz'])],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
        
        Node(
            package='humanoid_dummy',
            executable='humanoid_sqp_mpc',
            name='humanoid_sqp_mpc',
            output='screen',
            parameters=[{
                'multiplot': LaunchConfiguration('multiplot'),
                'taskFile': LaunchConfiguration('taskFile'),
                'referenceFile': LaunchConfiguration('referenceFile'),
                'urdfFile': LaunchConfiguration('urdfFile'),
                'urdfFileOrigin': LaunchConfiguration('urdfFileOrigin'),
                'gaitCommandFile': LaunchConfiguration('gaitCommandFile')
            }]
        ),
        
        Node(
            package='humanoid_dummy',
            executable='humanoid_dummy_node',
            name='humanoid_dummy_node',
            output='screen',
            prefix=['gnome-terminal', '--'],
            parameters=[{
                'taskFile': LaunchConfiguration('taskFile'),
                'referenceFile': LaunchConfiguration('referenceFile'),
                'urdfFile': LaunchConfiguration('urdfFile'),
                'urdfFileOrigin': LaunchConfiguration('urdfFileOrigin'),
                'gaitCommandFile': LaunchConfiguration('gaitCommandFile')
            }]
        ),
        
        Node(
            package='humanoid_dummy',
            executable='humanoid_target',
            name='humanoid_target',
            output='screen',
            prefix=['gnome-terminal', '--'],
            parameters=[{
                'taskFile': LaunchConfiguration('taskFile'),
                'referenceFile': LaunchConfiguration('referenceFile'),
                'urdfFile': LaunchConfiguration('urdfFile'),
                'urdfFileOrigin': LaunchConfiguration('urdfFileOrigin'),
                'gaitCommandFile': LaunchConfiguration('gaitCommandFile')
            }]
        ),
        
        Node(
            package='humanoid_dummy',
            executable='humanoid_gait_command',
            name='humanoid_gait_command',
            output='screen',
            prefix=['gnome-terminal', '--'],
            parameters=[{
                'taskFile': LaunchConfiguration('taskFile'),
                'referenceFile': LaunchConfiguration('referenceFile'),
                'urdfFile': LaunchConfiguration('urdfFile'),
                'urdfFileOrigin': LaunchConfiguration('urdfFileOrigin'),
                'gaitCommandFile': LaunchConfiguration('gaitCommandFile')
            }]
        )
    ])
