"""
Arm: state, kinematics, and motion primitives for any arm MoveIt drives.

Everything arm-specific (planning group, frames, joints, named poses) comes from
the ArmConfig, so the same calls move the Franka FER and the UR7e.
"""

from collections.abc import Callable, Mapping, Sequence
import xml.etree.ElementTree as ET

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    RobotTrajectory,
)
from moveit_msgs.srv import GetCartesianPath, GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

from arm_primitives._ros import call_action, call_service
from arm_primitives.config import ArmConfig
from arm_primitives.errors import PrimitiveError
from arm_primitives.geometry import BASE, offset_pose

ERROR_NAMES = {getattr(MoveItErrorCodes, n): n for n in dir(type(MoveItErrorCodes)) if n.isupper()}


def error_name(code: int) -> str:
    """Return the MoveItErrorCodes name for a code, e.g. -31 -> NO_IK_SOLUTION."""
    return ERROR_NAMES.get(code, str(code))


class Arm:
    """
    Motion primitives for one arm, through the move_group ROS API.

    Every move_* method plans, then executes unless execute=False, and returns the
    planned moveit_msgs/msg/RobotTrajectory (so a plan can be checked or replayed with
    execute()). Poses are Pose (in config.base_frame) or PoseStamped (in any frame
    MoveIt knows). Failures raise PrimitiveError.

    Subscribers
    -----------
    joint_states : sensor_msgs/msg/JointState
    robot_description_semantic : std_msgs/msg/String - SRDF, for its named poses

    Publishers
    ----------
    trajectory_execution_event : std_msgs/msg/String - "stop" halts execution

    Action Clients
    --------------
    move_action : moveit_msgs/action/MoveGroup
    execute_trajectory : moveit_msgs/action/ExecuteTrajectory

    Service Clients
    ---------------
    compute_fk : moveit_msgs/srv/GetPositionFK
    compute_ik : moveit_msgs/srv/GetPositionIK
    compute_cartesian_path : moveit_msgs/srv/GetCartesianPath
    """

    def __init__(
        self, node: Node, config: ArmConfig, callback_group: CallbackGroup | None = None
    ) -> None:
        """
        Create the ROS clients on the user's node.

        :param node: node shared by all components
        :param config: the arm's config
        :param callback_group: group for the clients; None = a new ReentrantCallbackGroup,
            so a primitive can be awaited from any callback
        """
        self.node = node
        self.config = config
        group = callback_group or ReentrantCallbackGroup()
        self._joint_state = JointState()
        self._positions: dict[str, float] = {}
        self._srdf_poses: dict[str, dict[str, float]] = {}
        self._state_waiters: list[Future[bool]] = []
        self._srdf_received = False
        self._srdf_waiters: list[Future[bool]] = []
        node.create_subscription(
            JointState, 'joint_states', self._joint_state_cb, 10, callback_group=group
        )
        node.create_subscription(
            String,
            'robot_description_semantic',
            self._srdf_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=group,
        )
        self._event_pub = node.create_publisher(String, 'trajectory_execution_event', 10)
        self._move_client = ActionClient(node, MoveGroup, 'move_action', callback_group=group)
        self._execute_client = ActionClient(
            node, ExecuteTrajectory, 'execute_trajectory', callback_group=group
        )
        self._fk_client = node.create_client(GetPositionFK, 'compute_fk', callback_group=group)
        self._ik_client = node.create_client(GetPositionIK, 'compute_ik', callback_group=group)
        self._cartesian_client = node.create_client(
            GetCartesianPath, 'compute_cartesian_path', callback_group=group
        )

    # ---------------------------------------------------------------- state

    @staticmethod
    def _release(waiters: list[Future[bool]]) -> None:
        """Wake everything waiting in _wait_until."""
        for waiter in waiters:
            if not waiter.done():
                waiter.set_result(True)
        waiters.clear()

    async def _wait_until(
        self, ready: Callable[[], bool], waiters: list[Future[bool]], timeout: float
    ) -> bool:
        """Return True as soon as ready() holds, or False after timeout seconds."""
        if ready():
            return True
        waiter: Future[bool] = Future(executor=self.node.executor)
        waiters.append(waiter)

        def expire() -> None:
            if not waiter.done():
                waiter.set_result(False)

        timer = self.node.create_timer(timeout, expire)
        try:
            return bool(await waiter)
        finally:
            # Cancel only: destroying it here races an executor thread that may already
            # hold the timer, and only happens while waiting at startup anyway.
            timer.cancel()

    def _joint_state_cb(self, msg: JointState) -> None:
        self._joint_state = msg
        self._positions.update(zip(msg.name, msg.position, strict=False))
        if self.has_joint_states():
            self._release(self._state_waiters)

    def _srdf_cb(self, msg: String) -> None:
        for state in ET.fromstring(msg.data).iter('group_state'):
            if state.get('group') == self.config.planning_group:
                self._srdf_poses[state.get('name', '')] = {
                    j.get('name', ''): float(j.get('value', '0')) for j in state.iter('joint')
                }
        self._srdf_received = True
        self._release(self._srdf_waiters)

    def has_joint_states(self) -> bool:
        """Return True once every joint of the planning group has a known position."""
        return all(j in self._positions for j in self.config.joints)

    async def wait_for_joint_states(self, timeout: float = 10.0) -> None:
        """
        Wait until joint_states has reported every joint of the planning group.

        :param timeout: seconds to wait
        :raises PrimitiveError: if the joints are not reported in time
        """
        if not await self._wait_until(self.has_joint_states, self._state_waiters, timeout):
            missing = [j for j in self.config.joints if j not in self._positions]
            raise PrimitiveError(f'no joint_states for {missing} after {timeout} s')

    async def wait_for_srdf(self, timeout: float = 5.0) -> bool:
        """
        Wait for move_group's SRDF (robot_description_semantic), which has the named poses.

        :param timeout: seconds to wait
        :return: True if the SRDF arrived
        """
        return await self._wait_until(lambda: self._srdf_received, self._srdf_waiters, timeout)

    def joint_positions(self) -> dict[str, float]:
        """Return the latest position of each joint of the planning group, in order."""
        if not self.has_joint_states():
            raise PrimitiveError('no joint_states received yet')
        return {j: self._positions[j] for j in self.config.joints}

    def joint_state(self) -> JointState:
        """Return the latest joint_states message (all joints, including fingers)."""
        return self._joint_state

    def named_poses(self) -> list[str]:
        """Return the names move_to_named() accepts: config named_poses, then SRDF ones."""
        return sorted(set(self.config.named_poses) | set(self._srdf_poses))

    def _full_state(self, joints: Mapping[str, float] | None = None) -> JointState:
        """Return the current joint state with the given joints replaced."""
        positions = dict(self._positions)
        if joints:
            positions.update(joints)
        return JointState(name=list(positions), position=list(positions.values()))

    def _stamped(self, pose: Pose | PoseStamped) -> PoseStamped:
        """Return the pose as a PoseStamped; no frame means config.base_frame."""
        if isinstance(pose, PoseStamped) and pose.header.frame_id:
            return pose
        stamped = PoseStamped(pose=pose.pose if isinstance(pose, PoseStamped) else pose)
        stamped.header.frame_id = self.config.base_frame
        return stamped

    # ----------------------------------------------------------- kinematics

    async def fk(
        self,
        joints: Mapping[str, float] | None = None,
        link: str | None = None,
        frame_id: str | None = None,
    ) -> Pose:
        """
        Compute the pose of a link.

        :param joints: joint positions to use; None = current, missing joints = current
        :param link: link to compute; None = config.ee_link
        :param frame_id: frame of the result; None = config.base_frame
        :return: geometry_msgs/msg/Pose of the link
        """
        request = GetPositionFK.Request()
        request.header.frame_id = frame_id or self.config.base_frame
        request.fk_link_names = [link or self.config.ee_link]
        request.robot_state.joint_state = self._full_state(joints)
        response = await call_service(self._fk_client, request)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise PrimitiveError(f'FK failed: {error_name(response.error_code.val)}')
        pose = response.pose_stamped[0].pose
        if not isinstance(pose, Pose):  # sequence fields are untyped in the generated code
            raise PrimitiveError('FK returned no pose')
        return pose

    async def ee_pose(self) -> Pose:
        """Return the current pose of config.ee_link in config.base_frame."""
        await self.wait_for_joint_states()
        return await self.fk()

    async def ik(
        self,
        pose: Pose | PoseStamped,
        seed: Mapping[str, float] | None = None,
        avoid_collisions: bool = True,
        timeout: float = 0.5,
    ) -> dict[str, float]:
        """
        Compute joint positions that put config.ee_link at a pose.

        :param pose: target pose (Pose = in config.base_frame)
        :param seed: joint positions to start the search from; None = current
        :param avoid_collisions: True = only accept collision-free solutions
        :param timeout: seconds the solver may take
        :return: joint name -> position for the planning group
        """
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = self.config.planning_group
        ik.ik_link_name = self.config.ee_link
        ik.pose_stamped = self._stamped(pose)
        ik.robot_state.joint_state = self._full_state(seed)
        ik.avoid_collisions = avoid_collisions
        ik.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1.0) * 1e9))
        response = await call_service(self._ik_client, request)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise PrimitiveError(f'IK failed: {error_name(response.error_code.val)}')
        solution = response.solution.joint_state
        found = dict(zip(solution.name, solution.position, strict=False))
        return {j: found[j] for j in self.config.joints if j in found}

    # ---------------------------------------------------------- execution

    def _scaling(self, velocity: float | None, acceleration: float | None) -> tuple[float, float]:
        v = self.config.velocity_scaling if velocity is None else velocity
        a = self.config.acceleration_scaling if acceleration is None else acceleration
        for name, value in (('velocity', v), ('acceleration', a)):
            if not 0.0 < value <= 1.0:
                raise PrimitiveError(f'{name} scaling must be in (0, 1], got {value}')
        return v, a

    async def execute(self, trajectory: RobotTrajectory) -> None:
        """
        Execute a planned trajectory.

        :param trajectory: moveit_msgs/msg/RobotTrajectory from a move_* call
        :raises PrimitiveError: if execution fails or is stopped
        """
        result = await call_action(
            self._execute_client,
            ExecuteTrajectory.Goal(trajectory=trajectory),
            'execute_trajectory',
        )
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise PrimitiveError(f'execution failed: {error_name(result.error_code.val)}')

    def stop(self) -> None:
        """Stop the trajectory being executed (the running move_* call then fails)."""
        self._event_pub.publish(String(data='stop'))

    async def _finish(
        self, trajectory: RobotTrajectory, execute: bool, what: str
    ) -> RobotTrajectory:
        points = len(trajectory.joint_trajectory.points)
        self.node.get_logger().info(f'{self.config.name}: planned {what} ({points} points)')
        if execute:
            await self.execute(trajectory)
        return trajectory

    async def _plan(
        self,
        constraints: Constraints,
        velocity_scaling: float | None,
        acceleration_scaling: float | None,
    ) -> RobotTrajectory:
        velocity, acceleration = self._scaling(velocity_scaling, acceleration_scaling)
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.config.planning_group
        request.pipeline_id = self.config.planning_pipeline
        request.planner_id = self.config.planner_id
        request.num_planning_attempts = self.config.planning_attempts
        request.allowed_planning_time = self.config.planning_time
        request.max_velocity_scaling_factor = velocity
        request.max_acceleration_scaling_factor = acceleration
        request.start_state.is_diff = True  # plan from the current state
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        result = await call_action(self._move_client, goal, 'move_action')
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise PrimitiveError(f'planning failed: {error_name(result.error_code.val)}')
        return result.planned_trajectory

    # -------------------------------------------------------------- motion

    async def move_to_joints(
        self,
        joints: Mapping[str, float] | Sequence[float],
        execute: bool = True,
        velocity_scaling: float | None = None,
        acceleration_scaling: float | None = None,
        tolerance: float = 1e-3,
    ) -> RobotTrajectory:
        """
        Move to joint positions.

        :param joints: joint name -> position, or one position per config.joints in order;
            joints of the group that are left out keep their current position
        :param execute: False = plan only
        :param velocity_scaling: fraction of max joint velocity; None = config default
        :param acceleration_scaling: fraction of max joint acceleration; None = config default
        :param tolerance: allowed error per joint (rad or m)
        :return: the planned trajectory
        """
        if isinstance(joints, Mapping):
            targets = {str(k): float(v) for k, v in joints.items()}
        else:
            if len(joints) != len(self.config.joints):
                raise PrimitiveError(
                    f'expected {len(self.config.joints)} positions for '
                    f'{list(self.config.joints)}, got {len(joints)}'
                )
            targets = dict(zip(self.config.joints, map(float, joints), strict=True))
        unknown = set(targets) - set(self.config.joints)
        if unknown:
            raise PrimitiveError(
                f'joints {sorted(unknown)} are not in {self.config.planning_group}'
            )
        constraints = Constraints()
        for name, position in targets.items():
            constraints.joint_constraints.append(
                JointConstraint(
                    joint_name=name,
                    position=position,
                    tolerance_above=tolerance,
                    tolerance_below=tolerance,
                    weight=1.0,
                )
            )
        trajectory = await self._plan(constraints, velocity_scaling, acceleration_scaling)
        return await self._finish(trajectory, execute, f'joint move {targets}')

    async def move_to_named(
        self,
        name: str,
        execute: bool = True,
        velocity_scaling: float | None = None,
        acceleration_scaling: float | None = None,
    ) -> RobotTrajectory:
        """
        Move to a named pose from config.named_poses or the SRDF (e.g. 'ready').

        :param name: pose name; see named_poses()
        :param execute: False = plan only
        :param velocity_scaling: fraction of max joint velocity; None = config default
        :param acceleration_scaling: fraction of max joint acceleration; None = config default
        :return: the planned trajectory
        """
        if name not in self.config.named_poses:
            await self.wait_for_srdf()  # SRDF poses arrive a moment after startup
        joints = self.config.named_poses.get(name) or self._srdf_poses.get(name)
        if joints is None:
            raise PrimitiveError(f'unknown named pose "{name}"; known: {self.named_poses()}')
        return await self.move_to_joints(joints, execute, velocity_scaling, acceleration_scaling)

    def pose_constraints(
        self,
        pose: Pose | PoseStamped,
        position_tolerance: float | None = None,
        orientation_tolerance: float | None = None,
    ) -> Constraints:
        """
        Build MoveIt goal constraints that put config.ee_link at a pose.

        :param pose: target pose (Pose = in config.base_frame)
        :param position_tolerance: radius in m around the target position; None = config
        :param orientation_tolerance: rad about each axis; None = config
        :return: moveit_msgs/msg/Constraints
        """
        stamped = self._stamped(pose)
        position_tol = position_tolerance or self.config.position_tolerance
        orientation_tol = orientation_tolerance or self.config.orientation_tolerance
        position = PositionConstraint(header=stamped.header, link_name=self.config.ee_link)
        position.constraint_region.primitives.append(
            SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[position_tol])
        )
        position.constraint_region.primitive_poses.append(Pose(position=stamped.pose.position))
        position.weight = 1.0
        orientation = OrientationConstraint(
            header=stamped.header,
            link_name=self.config.ee_link,
            orientation=stamped.pose.orientation,
            absolute_x_axis_tolerance=orientation_tol,
            absolute_y_axis_tolerance=orientation_tol,
            absolute_z_axis_tolerance=orientation_tol,
            parameterization=OrientationConstraint.ROTATION_VECTOR,
            weight=1.0,
        )
        return Constraints(position_constraints=[position], orientation_constraints=[orientation])

    async def move_to_pose(
        self,
        pose: Pose | PoseStamped,
        linear: bool = False,
        execute: bool = True,
        velocity_scaling: float | None = None,
        acceleration_scaling: float | None = None,
        avoid_collisions: bool = True,
    ) -> RobotTrajectory:
        """
        Move config.ee_link to a pose.

        :param pose: target pose (Pose = in config.base_frame)
        :param linear: True = straight-line Cartesian path, False = free-space plan
        :param execute: False = plan only
        :param velocity_scaling: fraction of max joint velocity; None = config default
        :param acceleration_scaling: fraction of max joint acceleration; None = config default
        :param avoid_collisions: linear only; False = skip collision checks on the path
        :return: the planned trajectory
        """
        if linear:
            return await self.move_cartesian(
                [pose],
                execute=execute,
                velocity_scaling=velocity_scaling,
                acceleration_scaling=acceleration_scaling,
                avoid_collisions=avoid_collisions,
            )
        constraints = self.pose_constraints(pose)
        trajectory = await self._plan(constraints, velocity_scaling, acceleration_scaling)
        return await self._finish(trajectory, execute, 'pose move')

    async def move_cartesian(
        self,
        waypoints: Sequence[Pose | PoseStamped],
        execute: bool = True,
        velocity_scaling: float | None = None,
        acceleration_scaling: float | None = None,
        avoid_collisions: bool = True,
        min_fraction: float | None = None,
    ) -> RobotTrajectory:
        """
        Move config.ee_link in straight lines through waypoints.

        :param waypoints: poses to pass through; all in the frame of the first one
        :param execute: False = plan only
        :param velocity_scaling: fraction of max joint velocity; None = config default
        :param acceleration_scaling: fraction of max joint acceleration; None = config default
        :param avoid_collisions: False = skip collision checks (e.g. when touching an object)
        :param min_fraction: fail if less of the path plans; None = config default
        :return: the planned trajectory
        """
        if not waypoints:
            raise PrimitiveError('move_cartesian needs at least one waypoint')
        velocity, acceleration = self._scaling(velocity_scaling, acceleration_scaling)
        stamped = [self._stamped(p) for p in waypoints]
        frame = stamped[0].header.frame_id
        if any(s.header.frame_id != frame for s in stamped):
            raise PrimitiveError('all Cartesian waypoints must be in the same frame')
        request = GetCartesianPath.Request()
        request.header.frame_id = frame
        request.start_state.is_diff = True
        request.group_name = self.config.planning_group
        request.link_name = self.config.ee_link
        request.waypoints = [s.pose for s in stamped]
        request.max_step = self.config.cartesian_step
        request.avoid_collisions = avoid_collisions
        request.max_velocity_scaling_factor = velocity
        request.max_acceleration_scaling_factor = acceleration
        response = await call_service(self._cartesian_client, request)
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise PrimitiveError(
                f'Cartesian planning failed: {error_name(response.error_code.val)}'
            )
        needed = self.config.min_cartesian_fraction if min_fraction is None else min_fraction
        if response.fraction < needed:
            raise PrimitiveError(
                f'Cartesian path only {response.fraction * 100:.1f}% feasible '
                '(collision, joint limit, or out of reach)'
            )
        return await self._finish(response.solution, execute, 'Cartesian move')

    async def move_relative(
        self,
        translation: Sequence[float] = (0.0, 0.0, 0.0),
        rotation: Sequence[float] = (0.0, 0.0, 0.0),
        frame: str = BASE,
        linear: bool = True,
        execute: bool = True,
        velocity_scaling: float | None = None,
        acceleration_scaling: float | None = None,
        avoid_collisions: bool = True,
    ) -> RobotTrajectory:
        """
        Move config.ee_link by an offset from its current pose.

        :param translation: (dx, dy, dz) in m
        :param rotation: (roll, pitch, yaw) in rad, about the end-effector's position
        :param frame: 'base' = along/about config.base_frame axes, 'tool' = ee_link axes
        :param linear: True = straight-line Cartesian path, False = free-space plan
        :param execute: False = plan only
        :param velocity_scaling: fraction of max joint velocity; None = config default
        :param acceleration_scaling: fraction of max joint acceleration; None = config default
        :param avoid_collisions: linear only; False = skip collision checks on the path
        :return: the planned trajectory
        """
        target = offset_pose(await self.ee_pose(), translation, rotation, frame)
        return await self.move_to_pose(
            target,
            linear=linear,
            execute=execute,
            velocity_scaling=velocity_scaling,
            acceleration_scaling=acceleration_scaling,
            avoid_collisions=avoid_collisions,
        )

    def tool_down_pose(self, x: float, y: float, z: float, yaw: float = 0.0) -> Pose:
        """
        Return a pose at (x, y, z) in config.base_frame with the tool pointing down.

        :param x: m
        :param y: m
        :param z: m
        :param yaw: rotation about the base z axis in rad
        :return: geometry_msgs/msg/Pose
        """
        down = Pose()
        down.orientation.x, down.orientation.y, down.orientation.z, down.orientation.w = (
            self.config.tool_down
        )
        return offset_pose(down, (x, y, z), (0.0, 0.0, yaw), BASE)
