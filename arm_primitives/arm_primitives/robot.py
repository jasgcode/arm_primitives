"""
Robot: an arm, its gripper and the planning scene, plus pick and place.

Typical script (works unchanged for every arm config)::

    from arm_primitives import Robot, run

    async def main(robot: Robot) -> None:
        await robot.arm.move_to_named('ready')
        pose = await robot.arm.ee_pose()
        await robot.arm.move_relative((0.0, 0.0, -0.05))

    run(main, arm='ur7e')
"""

from collections.abc import Callable, Coroutine

from geometry_msgs.msg import Pose, PoseStamped
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from arm_primitives.arm import Arm
from arm_primitives.config import ArmConfig, load_arm_config
from arm_primitives.geometry import BASE, offset_pose, TOOL
from arm_primitives.gripper import Gripper, make_gripper
from arm_primitives.scene import Scene


class Robot:
    """
    Everything needed to use one arm: robot.arm, robot.gripper, robot.scene.

    All three share the node and a ReentrantCallbackGroup, so primitives can be awaited
    from any callback when the node spins in a MultiThreadedExecutor.
    """

    def __init__(self, node: Node, config: ArmConfig | str) -> None:
        """
        Create the components for an arm.

        :param node: node shared by all components
        :param config: ArmConfig, or an arm name / YAML path for load_arm_config()
        """
        self.node = node
        self.config = load_arm_config(config) if isinstance(config, str) else config
        group = ReentrantCallbackGroup()
        self.arm = Arm(node, self.config, group)
        self.gripper: Gripper = make_gripper(node, self.config.gripper, group)
        self.scene = Scene(node, self.config, group)

    def _pose(self, pose: Pose | PoseStamped) -> tuple[Pose, str]:
        if isinstance(pose, PoseStamped):
            return pose.pose, pose.header.frame_id or self.config.base_frame
        return pose, self.config.base_frame

    async def pick(
        self,
        grasp: Pose | PoseStamped,
        object_name: str = '',
        approach: float | None = None,
        lift: float | None = None,
        width: float | None = None,
        force: float | None = None,
    ) -> None:
        """
        Pick an object up.

        Free-space move to `approach` m behind the grasp pose (along the tool z axis), open,
        straight line in, grasp, attach object_name (if given), straight up `lift` m.

        :param grasp: end-effector pose at the grasp (Pose = in config.base_frame)
        :param object_name: planning-scene object to attach; '' = none
        :param approach: approach distance in m; None = config.approach_distance
        :param lift: lift distance along the base z axis in m; None = config.lift_distance
        :param width: expected object width in m; None = close until contact
        :param force: grasp force in N; None = gripper default
        """
        pose, frame = self._pose(grasp)
        approach = self.config.approach_distance if approach is None else approach
        lift = self.config.lift_distance if lift is None else lift
        # The fingers touch the object (and the object its support) during the final
        # approach and the lift, so those straight lines skip collision checks when an
        # object is named.
        free = not object_name
        pre = offset_pose(pose, (0.0, 0.0, -approach), frame=TOOL)
        await self.arm.move_to_pose(_stamp(pre, frame))
        await self.gripper.open()
        await self.arm.move_to_pose(_stamp(pose, frame), linear=True, avoid_collisions=free)
        await self.gripper.grasp(width, force)
        if object_name:
            await self.scene.attach(object_name)
        await self.arm.move_relative((0.0, 0.0, lift), frame=BASE, avoid_collisions=free)

    async def place(
        self,
        release: Pose | PoseStamped,
        object_name: str = '',
        approach: float | None = None,
        lift: float | None = None,
    ) -> None:
        """
        Put the held object down: the reverse of pick().

        Free-space move to `lift` m above the release pose, straight line down, open,
        detach object_name (if given), straight back `approach` m along the tool z axis.

        :param release: end-effector pose when releasing (Pose = in config.base_frame)
        :param object_name: attached object to detach; '' = none
        :param approach: retreat distance along the tool z axis in m; None = config
        :param lift: height above the release pose to start from in m; None = config
        """
        pose, frame = self._pose(release)
        approach = self.config.approach_distance if approach is None else approach
        lift = self.config.lift_distance if lift is None else lift
        free = not object_name
        above = offset_pose(pose, (0.0, 0.0, lift), frame=BASE)
        await self.arm.move_to_pose(_stamp(above, frame))
        await self.arm.move_to_pose(_stamp(pose, frame), linear=True, avoid_collisions=free)
        await self.gripper.open()
        if object_name:
            await self.scene.detach(object_name)
        await self.arm.move_relative((0.0, 0.0, -approach), frame=TOOL, avoid_collisions=free)


def _stamp(pose: Pose, frame: str) -> PoseStamped:
    stamped = PoseStamped(pose=pose)
    stamped.header.frame_id = frame
    return stamped


def run(
    main: Callable[[Robot], Coroutine[object, object, None]],
    arm: str = '',
    node_name: str = 'arm_primitives_script',
    args: list[str] | None = None,
) -> None:
    """
    Run an async function with a ready Robot, then shut down.

    The arm comes from the ROS parameter "arm" if set (--ros-args -p arm:=ur7e),
    else from the arm argument.

    :param main: async function taking the Robot
    :param arm: arm name or YAML path used when the "arm" parameter is not set
    :param node_name: name of the node to create
    :param args: command line arguments for rclpy.init; None = sys.argv
    """
    # Ctrl-C raises KeyboardInterrupt here instead of rclpy shutting the context down
    # under the executor's threads.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node(node_name)
    executor = MultiThreadedExecutor()
    try:
        name = node.declare_parameter('arm', arm).get_parameter_value().string_value
        if not name:
            raise ValueError('no arm given: pass arm=... or --ros-args -p arm:=<name>')
        robot = Robot(node, name)
        executor.add_node(node)

        async def task() -> None:
            await robot.arm.wait_for_joint_states()
            await main(robot)

        future = executor.create_task(task)
        try:
            executor.spin_until_future_complete(future)
        except KeyboardInterrupt:
            robot.arm.stop()
            future.cancel()
            return
        exception = future.exception()
        if exception is not None:
            raise exception
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
