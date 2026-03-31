from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def find_workspace_setup():
    here = os.path.dirname(os.path.abspath(__file__))
    # Installed layout: install/<pkg>/share/<pkg>/launch/ -> 4 up = install/
    # Source layout: src/<pkg>/launch/ -> 3 up + /install
    for candidate in [
        os.path.abspath(os.path.join(here, '..', '..', '..', '..', 'setup.bash')),
        os.path.abspath(os.path.join(here, '..', '..', '..', 'install', 'setup.bash')),
    ]:
        install_dir = os.path.dirname(candidate)
        if os.path.exists(candidate) and (
            os.path.exists(os.path.join(install_dir, 'local_setup.bash'))
            or os.path.exists(os.path.join(install_dir, '_local_setup_util_sh.py'))
        ):
            return candidate

    for env_key in ('COLCON_PREFIX_PATH', 'AMENT_PREFIX_PATH'):
        for prefix in os.environ.get(env_key, '').split(':'):
            current = prefix
            while current:
                candidate = os.path.join(current, 'setup.bash')
                local_setup = os.path.join(current, 'local_setup.bash')
                local_setup_util = os.path.join(current, '_local_setup_util_sh.py')
                if os.path.basename(current) == 'install' and os.path.exists(candidate) and (
                    os.path.exists(local_setup) or os.path.exists(local_setup_util)
                ) and os.path.isdir(os.path.join(current, 'humanoid_controllers')):
                    return candidate
                parent = os.path.dirname(current)
                if parent == current:
                    break
                current = parent
    return ''


def find_ocs2_setup():
    candidate = os.path.join(os.path.expanduser('~'), 'ocs2_ws', 'install', 'setup.bash')
    return candidate if os.path.exists(candidate) else ''


def generate_launch_description():
    setup_path = find_workspace_setup()
    ocs2_setup_path = find_ocs2_setup()
    gnome_terminal_real = '/usr/bin/gnome-terminal.real'
    gnome_terminal = '/usr/bin/gnome-terminal'
    gnome_terminal_bin = gnome_terminal_real if os.path.exists(gnome_terminal_real) else gnome_terminal

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
                os.path.join(os.path.expanduser('~'), 'ocs2_ws/install/ocs2_ros_interfaces/lib'),
            ])
        ),
    ]

    setup_commands = []
    if ocs2_setup_path:
        setup_commands.append(f'source "{ocs2_setup_path}"')
    if setup_path:
        setup_commands.append(f'source "{setup_path}"')

    ros2_command_parts = []
    if setup_commands:
        ros2_command_parts.append(' && '.join(setup_commands))
    ros2_command_parts.append('stdbuf -oL -eL ros2 run humanoid_dummy humanoid_gait_command --ros-args')
    ros2_command = ' && '.join(ros2_command_parts)

    if os.path.exists(gnome_terminal_bin):
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
            # Keep transport consistent with controller/sim launches.
            'FASTDDS_BUILTIN_TRANSPORTS': 'UDPv4',
            'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', ''),
        }
        env_args = [f'{key}={value}' for key, value in sanitized_env.items() if value != '']
        launch_actions.append(
            ExecuteProcess(
                cmd=[
                    '/usr/bin/env',
                    '-i',
                    *env_args,
                    gnome_terminal_bin,
                    '--title=Humanoid Gait Command',
                    '--',
                    '/bin/bash',
                    '-lc',
                    [
                        'printf "Humanoid gait command terminal ready.\\n"; ',
                        'printf "Type list to show gaits. Walking also needs nonzero /cmd_vel.\\n\\n"; ',
                        ros2_command,
                        ' -r __node:=humanoid_gait_command -p gaitCommandFile:=', LaunchConfiguration('gaitCommandFile'),
                        '; status=$?; '
                        'printf "\\nHumanoid gait command exited with status %s.\\n" "$status"; '
                        'exec bash',
                    ],
                ],
                output='screen',
            )
        )
    else:
        launch_actions.append(
            ExecuteProcess(
                cmd=[
                    '/bin/bash',
                    '-lc',
                    [
                        'export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; ',
                        'if [ -r /dev/tty ] && [ -w /dev/tty ]; then '
                        'exec </dev/tty >/dev/tty 2>/dev/tty; '
                        'else '
                        'printf "WARNING: /dev/tty is unavailable, keyboard input may not work.\\n"; '
                        'fi; ',
                        'printf "Humanoid gait command running in current terminal (no gnome-terminal).\\n"; ',
                        'printf "Type list to show gaits. Walking also needs nonzero /cmd_vel.\\n"; ',
                        'printf "If robot still does not move, ensure controller/sim is running and publish nonzero /cmd_vel.\\n\\n"; ',
                        ros2_command,
                        ' -r __node:=humanoid_gait_command -p gaitCommandFile:=', LaunchConfiguration('gaitCommandFile'),
                    ],
                ],
                output='screen',
                emulate_tty=True,
            )
        )

    return LaunchDescription(launch_actions)
