"""
Bring up any configured arm (mock or real hardware) and the primitives server.

What starts for each arm and mode is the "bringup" section of config/arms/<arm>.yaml,
so a new arm needs a YAML file, not a new launch file.

ros2 launch arm_primitives arm.launch.py arm:=fer
ros2 launch arm_primitives arm.launch.py arm:=ur7e rviz:=false
ros2 launch arm_primitives arm.launch.py arm:=ur7e hardware:=real robot_ip:=192.168.1.102
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from arm_primitives.config import available_arms, HARDWARE_MODES, load_arm_config


def setup(context: LaunchContext) -> list[LaunchDescriptionEntity]:
    """Include the arm's bringup launch files and start the primitives server."""
    arm = LaunchConfiguration('arm').perform(context)
    hardware = LaunchConfiguration('hardware').perform(context)
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    values = {
        'robot_ip': robot_ip,
        'rviz': LaunchConfiguration('rviz').perform(context).lower(),
    }
    config = load_arm_config(arm)
    if hardware not in config.bringup:
        raise RuntimeError(f'arm "{config.name}" has no bringup for hardware "{hardware}"')
    if hardware == 'real' and not robot_ip:
        raise RuntimeError('hardware:=real needs robot_ip:=<address of the robot>')
    entities: list[LaunchDescriptionEntity] = []
    for include in config.bringup[hardware]:
        try:
            share = Path(get_package_share_directory(include.package))
        except PackageNotFoundError as error:
            raise RuntimeError(
                f'{config.name} {hardware} needs the package "{include.package}"; '
                'install or build it (see the arm_primitives README)'
            ) from error
        args = {}
        for key, value in include.args.items():
            for name, filled in values.items():
                value = value.replace(f'{{{name}}}', filled)
            args[key] = value
        entities.append(
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(str(share / 'launch' / include.launch)),
                launch_arguments=args.items(),
            )
        )
    if LaunchConfiguration('server').perform(context).lower() == 'true':
        entities.append(
            Node(
                package='arm_primitives',
                executable='primitives_node',
                parameters=[{'arm': arm}],
                output='screen',
            )
        )
    return entities


def generate_launch_description() -> LaunchDescription:
    """Declare the arguments and defer the includes to setup()."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'arm', description=f'Arm name or YAML path; installed: {available_arms()}'
            ),
            DeclareLaunchArgument(
                'hardware',
                default_value='mock',
                choices=list(HARDWARE_MODES),
                description='mock or real',
            ),
            DeclareLaunchArgument(
                'robot_ip', default_value='', description='Robot address (real)'
            ),
            DeclareLaunchArgument('rviz', default_value='true', description='Start rviz'),
            DeclareLaunchArgument(
                'server', default_value='true', description='Start the primitives service server'
            ),
            OpaqueFunction(function=setup),
        ]
    )
