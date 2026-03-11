from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def find_workspace_setup():
    workspace_setup = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', '..', '..', 'install', 'setup.bash')
    )
    workspace_install = os.path.dirname(workspace_setup)
    if os.path.exists(workspace_setup) and (
        os.path.exists(os.path.join(workspace_install, 'local_setup.bash'))
        or os.path.exists(os.path.join(workspace_install, '_local_setup_util_sh.py'))
    ):
        return workspace_setup

    for env_key in ('COLCON_PREFIX_PATH', 'AMENT_PREFIX_PATH'):
        for prefix in os.environ.get(env_key, '').split(':'):
            current = prefix
            while current:
                candidate = os.path.join(current, 'setup.bash')
                local_setup = os.path.join(current, 'local_setup.bash')
                local_setup_util = os.path.join(current, '_local_setup_util_sh.py')
                if os.path.basename(current) == 'install' and os.path.exists(candidate) and (
                    os.path.exists(local_setup) or os.path.exists(local_setup_util)
                ):
                    return candidate
                parent = os.path.dirname(current)
                if parent == current:
                    break
                current = parent
    return ''


def find_ocs2_setup():
    candidate = os.path.join(os.path.expanduser('~'), 'ocs2_ws', 'install', 'setup.bash')
    return candidate if os.path.exists(candidate) else ''


def create_gait_terminal_action():
    gnome_terminal_real = '/usr/bin/gnome-terminal.real'
    setup_path = find_workspace_setup()
    ocs2_setup_path = find_ocs2_setup()
    if not os.path.exists(gnome_terminal_real) or not setup_path:
        return None

    sanitized_env = {
        'HOME': os.environ.get('HOME', ''),
        'USER': os.environ.get('USER', ''),
        'LOGNAME': os.environ.get('LOGNAME', ''),
        'SHELL': os.environ.get('SHELL', '/bin/bash'),
        'PATH': '/usr/bin:/bin',
        'DISPLAY': os.environ.get('DISPLAY', ''),
        'XAUTHORITY': os.environ.get('XAUTHORITY', ''),
        'DBUS_SESSION_BUS_ADDRESS': os.environ.get('DBUS_SESSION_BUS_ADDRESS', ''),
        'XDG_RUNTIME_DIR': os.environ.get('XDG_RUNTIME_DIR', ''),
        'XDG_CURRENT_DESKTOP': os.environ.get('XDG_CURRENT_DESKTOP', ''),
        'XDG_SESSION_TYPE': os.environ.get('XDG_SESSION_TYPE', ''),
    }
    env_args = [f'{key}={value}' for key, value in sanitized_env.items() if value != '']

    setup_commands = []
    if ocs2_setup_path:
        setup_commands.append(f'source "{ocs2_setup_path}"')
    setup_commands.append(f'source "{setup_path}"')
    return ExecuteProcess(
        cmd=[
            '/usr/bin/env',
            '-i',
            *env_args,
            gnome_terminal_real,
            '--title=Humanoid Gait Command',
            '--',
            '/bin/bash',
            '-lc',
            [
                'printf "Humanoid gait command terminal ready.\\n"; ',
                'printf "Type list to show gaits. Walking also needs nonzero /cmd_vel.\\n\\n"; ',
                ' && '.join(setup_commands),
                ' && stdbuf -oL -eL ros2 run humanoid_dummy humanoid_gait_command --ros-args',
                ' -r __node:=humanoid_gait_command -p gaitCommandFile:=', LaunchConfiguration('gaitCommandFile'),
                '; status=$?; '
                'printf "\\nHumanoid gait command exited with status %s.\\n" "$status"; '
                'exec bash',
            ],
        ],
        output='screen',
    )


def generate_launch_description():
    launch_actions = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('taskFile', default_value=PathJoinSubstitution([FindPackageShare('humanoid_interface'), 'config/mpc/task.info'])),
        DeclareLaunchArgument('referenceFile', default_value=PathJoinSubstitution([FindPackageShare('humanoid_interface'), 'config/command/reference.info'])),
        DeclareLaunchArgument('urdfFile', default_value=PathJoinSubstitution([FindPackageShare('humanoid_legged_description'), 'urdf/humanoid_legged_control.urdf'])),
        DeclareLaunchArgument('urdfFileOrigin', default_value=PathJoinSubstitution([FindPackageShare('humanoid_legged_description'), 'urdf/humanoid_legged_origin.urdf'])),
        DeclareLaunchArgument('gaitCommandFile', default_value=PathJoinSubstitution([FindPackageShare('humanoid_interface'), 'config/command/gait.info'])),
        AppendEnvironmentVariable(
            'LD_LIBRARY_PATH',
            ':'.join([
                os.path.join(os.path.expanduser('~'), 'ocs2_ws/install/hpipm_catkin/lib'),
                os.path.join(os.path.expanduser('~'), 'ocs2_ws/install/hpp-fcl/lib'),
                os.path.join(os.path.expanduser('~'), 'ocs2_ws/install/ocs2_pinocchio_interface/lib'),
                os.path.join(os.path.expanduser('~'), 'ocs2_ws/install/ocs2_core/lib'),
                os.path.join(os.path.expanduser('~'), 'ocs2_ws/install/pinocchio/lib'),
                os.path.join(os.path.expanduser('~'), 'ocs2_ws/install/blasfeo_catkin/lib'),
                os.path.join(os.path.expanduser('~'), 'ocs2_ws/install/ocs2_msgs/lib'),
                os.path.join(os.path.expanduser('~'), 'ocs2_ws/install/ocs2_ros_interfaces/lib')
            ])
        ),
    ]

    gait_terminal_action = create_gait_terminal_action()
    if gait_terminal_action is not None:
        launch_actions.append(gait_terminal_action)

    launch_actions.extend([
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
                'gaitCommandFile': LaunchConfiguration('gaitCommandFile'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        Node(
            package='humanoid_controllers',
            executable='cheat_controller_node',
            name='cheat_controller',
            output='screen',
            prefix='gdb -q -batch -ex run -ex "thread apply all bt" --args',
            parameters=[{
                'taskFile': LaunchConfiguration('taskFile'),
                'referenceFile': LaunchConfiguration('referenceFile'),
                'urdfFile': LaunchConfiguration('urdfFile'),
                'urdfFileOrigin': LaunchConfiguration('urdfFileOrigin'),
                'gaitCommandFile': LaunchConfiguration('gaitCommandFile'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        Node(
            package='mujoco_sim',
            executable='humanoid_sim.py',
            name='humanoid_sim',
            output='screen',
            additional_env={'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4'},
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        Node(
            package='mujoco_sim',
            executable='teleop.py',
            name='teleop',
            output='screen',
            additional_env={'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4'},
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])

    return LaunchDescription(launch_actions)
