"""
Arm configuration: everything that differs between arms, loaded from YAML.

The primitives only ever read an ArmConfig, so supporting a new arm means writing
a new config/arms/<name>.yaml (see fer.yaml and ur7e.yaml), not new code.
"""

from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

from arm_primitives.geometry import normalize, Quat

PACKAGE = 'arm_primitives'
GRIPPER_TYPES = ('none', 'franka', 'gripper_command')
HARDWARE_MODES = ('mock', 'real')


class ConfigError(ValueError):
    """An arm config file is missing a key or has a value of the wrong type."""


@dataclass(frozen=True)
class GripperConfig:
    """
    How to drive the gripper on the arm.

    type 'none': no gripper; gripper primitives raise GripperError.
    type 'franka': franka_gripper node, franka_msgs actions <namespace>/move, grasp, homing.
    type 'gripper_command': any control_msgs/action/GripperCommand server at <namespace>
        (e.g. a Robotiq or ros2_control parallel gripper). Widths map linearly onto
        [closed_position, open_position].
    """

    type: str = 'none'
    namespace: str = ''  # action namespace (franka) or full action name (gripper_command)
    max_width: float = 0.08  # m, fully open
    min_width: float = 0.0  # m, fully closed
    speed: float = 0.05  # m/s
    force: float = 20.0  # N, default grasp force
    epsilon: float = 0.01  # m, franka: allowed object width error when grasping
    open_position: float = 0.0  # gripper_command: position for max_width
    closed_position: float = 0.0  # gripper_command: position for min_width


@dataclass(frozen=True)
class LaunchInclude:
    """One launch file to include to bring the arm up."""

    package: str
    launch: str  # file name inside <package>/launch
    args: Mapping[str, str] = field(default_factory=dict)  # '{robot_ip}' etc. are substituted


@dataclass(frozen=True)
class ArmConfig:
    """Names, frames, defaults and bringup for one arm."""

    name: str
    planning_group: str  # MoveIt planning group the primitives plan for
    base_frame: str  # frame poses are expressed in unless a message says otherwise
    ee_link: str  # link that pose goals and Cartesian paths move
    joints: tuple[str, ...]  # joints of the planning group, in order
    description: str = ''
    attach_link: str = ''  # link attached objects move with ('' = ee_link)
    touch_links: tuple[str, ...] = ()  # links allowed to touch an attached object
    tool_down: Quat = (1.0, 0.0, 0.0, 0.0)  # ee orientation with the tool z axis down
    named_poses: Mapping[str, Mapping[str, float]] = field(default_factory=dict)
    velocity_scaling: float = 0.2  # default fraction of max joint velocity (0, 1]
    acceleration_scaling: float = 0.2  # default fraction of max joint acceleration (0, 1]
    planning_time: float = 5.0  # s
    planning_attempts: int = 10
    planning_pipeline: str = ''  # '' = move_group default
    planner_id: str = ''  # '' = pipeline default
    position_tolerance: float = 0.002  # m, pose goals
    orientation_tolerance: float = 0.01  # rad, pose goals
    cartesian_step: float = 0.005  # m between Cartesian path points
    min_cartesian_fraction: float = 0.99  # fail a Cartesian move that plans less than this
    approach_distance: float = 0.1  # m, pick/place approach along the tool z axis
    lift_distance: float = 0.1  # m, pick/place lift along the base z axis
    gripper: GripperConfig = field(default_factory=GripperConfig)
    bringup: Mapping[str, tuple[LaunchInclude, ...]] = field(default_factory=dict)

    @property
    def attach(self) -> str:
        """Link attached objects move with: attach_link, or ee_link if unset."""
        return self.attach_link or self.ee_link


def _str(data: Mapping[str, object], key: str, default: str | None = None) -> str:
    value = data.get(key, default)
    if not isinstance(value, str):
        raise ConfigError(f'"{key}" must be a string, got {value!r}')
    return value


def _float(data: Mapping[str, object], key: str, default: float) -> float:
    value = data.get(key, default)
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ConfigError(f'"{key}" must be a number, got {value!r}')
    return float(value)


def _int(data: Mapping[str, object], key: str, default: int) -> int:
    value = data.get(key, default)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(f'"{key}" must be an integer, got {value!r}')
    return value


def _mapping(data: Mapping[str, object], key: str) -> Mapping[str, object]:
    value = data.get(key, {})
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ConfigError(f'"{key}" must be a mapping, got {value!r}')
    return {str(k): v for k, v in value.items()}


def _strings(data: Mapping[str, object], key: str) -> tuple[str, ...]:
    value = data.get(key, [])
    if not isinstance(value, list) or not all(isinstance(v, str) for v in value):
        raise ConfigError(f'"{key}" must be a list of strings, got {value!r}')
    return tuple(str(v) for v in value)


def _numbers(value: object, key: str) -> tuple[float, ...]:
    if not isinstance(value, list) or not all(
        isinstance(v, int | float) and not isinstance(v, bool) for v in value
    ):
        raise ConfigError(f'"{key}" must be a list of numbers, got {value!r}')
    return tuple(float(v) for v in value)


def _gripper(data: Mapping[str, object]) -> GripperConfig:
    default = GripperConfig()
    gripper_type = _str(data, 'type', default.type)
    if gripper_type not in GRIPPER_TYPES:
        raise ConfigError(f'gripper "type" must be one of {GRIPPER_TYPES}, got "{gripper_type}"')
    config = GripperConfig(
        type=gripper_type,
        namespace=_str(data, 'namespace', default.namespace),
        max_width=_float(data, 'max_width', default.max_width),
        min_width=_float(data, 'min_width', default.min_width),
        speed=_float(data, 'speed', default.speed),
        force=_float(data, 'force', default.force),
        epsilon=_float(data, 'epsilon', default.epsilon),
        open_position=_float(data, 'open_position', default.open_position),
        closed_position=_float(data, 'closed_position', default.closed_position),
    )
    if config.type != 'none' and not config.namespace:
        raise ConfigError(f'gripper type "{config.type}" needs a "namespace"')
    if config.max_width <= config.min_width:
        raise ConfigError('gripper "max_width" must be larger than "min_width"')
    return config


def _bringup(data: Mapping[str, object]) -> dict[str, tuple[LaunchInclude, ...]]:
    bringup: dict[str, tuple[LaunchInclude, ...]] = {}
    for mode, includes in data.items():
        if mode not in HARDWARE_MODES:
            raise ConfigError(f'bringup modes must be in {HARDWARE_MODES}, got "{mode}"')
        if not isinstance(includes, list):
            raise ConfigError(f'bringup "{mode}" must be a list of launch includes')
        parsed = []
        for include in includes:
            if not isinstance(include, Mapping):
                raise ConfigError(f'bringup "{mode}" entries must be mappings, got {include!r}')
            entry = {str(k): v for k, v in include.items()}
            args = _mapping(entry, 'args')
            parsed.append(
                LaunchInclude(
                    package=_str(entry, 'package'),
                    launch=_str(entry, 'launch'),
                    args={k: _launch_arg(v) for k, v in args.items()},
                )
            )
        bringup[mode] = tuple(parsed)
    return bringup


def _launch_arg(value: object) -> str:
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


def parse_arm_config(data: Mapping[str, object]) -> ArmConfig:
    """
    Build an ArmConfig from the contents of an arm YAML file.

    :param data: the parsed YAML mapping
    :return: the validated ArmConfig
    :raises ConfigError: if a key is missing or has the wrong type
    """
    default = ArmConfig(name='', planning_group='', base_frame='', ee_link='', joints=())
    joints = _strings(data, 'joints')
    if not joints:
        raise ConfigError('"joints" must list the joints of the planning group')
    named: dict[str, dict[str, float]] = {}
    for pose_name, pose in _mapping(data, 'named_poses').items():
        if not isinstance(pose, Mapping):
            raise ConfigError(f'named pose "{pose_name}" must map joint names to values')
        values = {str(k): v for k, v in pose.items()}
        unknown = set(values) - set(joints)
        if unknown:
            raise ConfigError(f'named pose "{pose_name}" has unknown joints {sorted(unknown)}')
        named[pose_name] = {k: _float(values, k, 0.0) for k in values}
    tool_down = _numbers(data.get('tool_down', list(default.tool_down)), 'tool_down')
    if len(tool_down) != 4:
        raise ConfigError('"tool_down" must be a quaternion [x, y, z, w]')
    config = ArmConfig(
        name=_str(data, 'name'),
        description=_str(data, 'description', ''),
        planning_group=_str(data, 'planning_group'),
        base_frame=_str(data, 'base_frame'),
        ee_link=_str(data, 'ee_link'),
        joints=joints,
        attach_link=_str(data, 'attach_link', ''),
        touch_links=_strings(data, 'touch_links'),
        tool_down=normalize((tool_down[0], tool_down[1], tool_down[2], tool_down[3])),
        named_poses=named,
        velocity_scaling=_float(data, 'velocity_scaling', default.velocity_scaling),
        acceleration_scaling=_float(data, 'acceleration_scaling', default.acceleration_scaling),
        planning_time=_float(data, 'planning_time', default.planning_time),
        planning_attempts=_int(data, 'planning_attempts', default.planning_attempts),
        planning_pipeline=_str(data, 'planning_pipeline', default.planning_pipeline),
        planner_id=_str(data, 'planner_id', default.planner_id),
        position_tolerance=_float(data, 'position_tolerance', default.position_tolerance),
        orientation_tolerance=_float(data, 'orientation_tolerance', default.orientation_tolerance),
        cartesian_step=_float(data, 'cartesian_step', default.cartesian_step),
        min_cartesian_fraction=_float(
            data, 'min_cartesian_fraction', default.min_cartesian_fraction
        ),
        approach_distance=_float(data, 'approach_distance', default.approach_distance),
        lift_distance=_float(data, 'lift_distance', default.lift_distance),
        gripper=_gripper(_mapping(data, 'gripper')),
        bringup=_bringup(_mapping(data, 'bringup')),
    )
    for scaling in (config.velocity_scaling, config.acceleration_scaling):
        if not 0.0 < scaling <= 1.0:
            raise ConfigError('velocity/acceleration scaling must be in (0, 1]')
    return config


def config_path(arm: str) -> Path:
    """
    Return the YAML file for an arm.

    :param arm: a path to a YAML file, or the name of one in arm_primitives/config/arms
    :return: path to the file
    """
    path = Path(arm).expanduser()
    if path.suffix in ('.yaml', '.yml'):
        return path
    return Path(get_package_share_directory(PACKAGE)) / 'config' / 'arms' / f'{arm}.yaml'


def available_arms() -> list[str]:
    """Return the names of the arm configs installed with arm_primitives."""
    arms = Path(get_package_share_directory(PACKAGE)) / 'config' / 'arms'
    return sorted(p.stem for p in arms.glob('*.yaml'))


def load_arm_config(arm: str) -> ArmConfig:
    """
    Load and validate an arm config.

    :param arm: arm name (e.g. 'fer', 'ur7e') or path to a YAML file
    :return: the ArmConfig
    :raises ConfigError: if the file is missing or invalid
    """
    path = config_path(arm)
    if not path.is_file():
        raise ConfigError(f'no arm config "{arm}" ({path}); installed arms: {available_arms()}')
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, Mapping):
        raise ConfigError(f'{path} must contain a YAML mapping')
    try:
        return parse_arm_config({str(k): v for k, v in data.items()})
    except ConfigError as error:
        raise ConfigError(f'{path}: {error}') from error
