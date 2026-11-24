"""
Any UR arm on ros2_control mock hardware with MoveIt, without ur_robot_driver.

Starts robot_state_publisher, ros2_control_node (mock_components/GenericSystem from
ur_description's ur_mocked.urdf.xacro), the joint_state_broadcaster and
joint_trajectory_controller, move_group (ur_moveit_config), and optionally rviz.

ros2 launch arm_primitives ur_mock.launch.py ur_type:=ur7e rviz:=false
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def setup(context: LaunchContext) -> list[Node]:
    """Build the nodes once ur_type is known (the MoveIt config needs it as a string)."""
    ur_type = LaunchConfiguration('ur_type').perform(context)
    description = Path(get_package_share_directory('ur_description'))
    moveit_config = (
        MoveItConfigsBuilder('ur', package_name='ur_moveit_config')
        .robot_description(
            file_path=str(description / 'urdf' / 'ur_mocked.urdf.xacro'),
            mappings={'ur_type': ur_type, 'name': 'ur'},
        )
        .robot_description_semantic(file_path='srdf/ur.srdf.xacro', mappings={'name': 'ur'})
        .to_moveit_configs()
    )
    controllers = (
        Path(get_package_share_directory('arm_primitives'))
        / 'config'
        / 'controllers'
        / 'ur_mock_controllers.yaml'
    )
    rviz_config = Path(get_package_share_directory('ur_moveit_config')) / 'config' / 'moveit.rviz'
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[moveit_config.robot_description],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[str(controllers)],
            remappings=[('~/robot_description', '/robot_description')],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            # Controllers read their parameters from --param-file, not the manager's file
            arguments=[
                'joint_state_broadcaster',
                'joint_trajectory_controller',
                '--param-file',
                str(controllers),
            ],
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            parameters=[moveit_config.to_dict(), {'publish_robot_description_semantic': True}],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(rviz_config)],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='log',
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    """Declare the arguments and defer node creation to setup()."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'ur_type', default_value='ur7e', description='UR model, e.g. ur7e'
            ),
            DeclareLaunchArgument('rviz', default_value='true', description='Start rviz'),
            OpaqueFunction(function=setup),
        ]
    )
