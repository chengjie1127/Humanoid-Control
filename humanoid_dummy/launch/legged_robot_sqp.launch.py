from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='humanoid'),
        DeclareLaunchArgument('taskFile', default_value=PathJoinSubstitution([FindPackageShare('humanoid_interface'), 'config/mpc/task.info'])),
        DeclareLaunchArgument('referenceFile', default_value=PathJoinSubstitution([FindPackageShare('humanoid_interface'), 'config/command/reference.info'])),
        DeclareLaunchArgument('urdfFile', default_value=PathJoinSubstitution([FindPackageShare('humanoid_legged_description'), 'urdf/humanoid_legged_control.urdf'])),
        DeclareLaunchArgument('urdfFileOrigin', default_value=PathJoinSubstitution([FindPackageShare('humanoid_legged_description'), 'urdf/humanoid_legged_origin.urdf'])),
        DeclareLaunchArgument('gaitCommandFile', default_value=PathJoinSubstitution([FindPackageShare('humanoid_interface'), 'config/command/gait.info'])),
        
        # Append OCS2 library paths to LD_LIBRARY_PATH so the nodes can find them at runtime
        AppendEnvironmentVariable(
            'LD_LIBRARY_PATH',
            ':'.join([
                os.path.join(os.path.expanduser('~'), 'ocs2_ws/install/hpipm_catkin/lib'),
                os.path.join(os.path.expanduser('~'), 'ocs2_ws/install/hpp-fcl/lib'),
                os.path.join(os.path.expanduser('~'), 'ocs2_ws/install/ocs2_self_collision/lib'),
                os.path.join(os.path.expanduser('~'), 'ocs2_ws/install/ocs2_self_collision_visualization/lib'),
                os.path.join(os.path.expanduser('~'), 'ocs2_ws/install/ocs2_pinocchio_interface/lib'),
                os.path.join(os.path.expanduser('~'), 'ocs2_ws/install/ocs2_core/lib'),
                os.path.join(os.path.expanduser('~'), 'ocs2_ws/install/pinocchio/lib'),
                os.path.join(os.path.expanduser('~'), 'ocs2_ws/install/blasfeo_catkin/lib'),
                os.path.join(os.path.expanduser('~'), 'ocs2_ws/install/ocs2_msgs/lib'),
                os.path.join(os.path.expanduser('~'), 'ocs2_ws/install/ocs2_ros_interfaces/lib')
            ])
        ),
        
        GroupAction([
            PushRosNamespace(LaunchConfiguration('namespace')),

            # Include RViz from humanoid-legged-description
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare('humanoid_legged_description'), 'launch', 'display.launch.py'])
                )
            ),

            Node(
                package='humanoid_dummy',
                executable='humanoid_sqp_mpc',
                name='humanoid_sqp_mpc',
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
                package='humanoid_dummy',
                executable='humanoid_dummy_node',
                name='humanoid_dummy_node',
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
                package='humanoid_dummy',
                executable='humanoid_target',
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
                }]
            )
        ])
    ])
