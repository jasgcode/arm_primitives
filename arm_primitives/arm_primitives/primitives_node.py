"""
ROS services for every primitive, for any configured arm.

Parameters
----------
arm : str - arm name (fer, ur7e, ...) or path to an arm YAML file

Services (all under ~/, i.e. /arm_primitives/<name>)
--------
move_to_named : arm_primitives_interfaces/srv/MoveToNamed
move_to_joints : arm_primitives_interfaces/srv/MoveToJoints
move_to_pose : arm_primitives_interfaces/srv/MoveToPose
move_relative : arm_primitives_interfaces/srv/MoveRelative
gripper : arm_primitives_interfaces/srv/Gripper
scene : arm_primitives_interfaces/srv/SceneObject
pick_place : arm_primitives_interfaces/srv/PickPlace
get_state : arm_primitives_interfaces/srv/GetArmState
stop : std_srvs/srv/Trigger - halt the running motion (served while a move is running)

"""

from collections.abc import Awaitable

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from arm_primitives.errors import PrimitiveError
from arm_primitives.robot import Robot
from arm_primitives_interfaces.srv import (
    GetArmState,
    Gripper,
    MoveRelative,
    MoveToJoints,
    MoveToNamed,
    MoveToPose,
    PickPlace,
    SceneObject,
)


def _positive(value: float) -> float | None:
    """Return None (use the default) for values <= 0, else the value."""
    return value if value > 0.0 else None


class PrimitivesNode(Node):
    """Serve the primitives of one arm."""

    def __init__(self) -> None:
        super().__init__('arm_primitives')
        arm = self.declare_parameter('arm', '').get_parameter_value().string_value
        if not arm:
            raise ValueError('set the "arm" parameter, e.g. --ros-args -p arm:=fer')
        self.robot = Robot(self, arm)
        self.get_logger().info(
            f'arm_primitives for "{self.robot.config.name}": group '
            f'{self.robot.config.planning_group}, base {self.robot.config.base_frame}, '
            f'ee {self.robot.config.ee_link}, gripper {self.robot.config.gripper.type}'
        )
        motion = MutuallyExclusiveCallbackGroup()  # one primitive at a time
        self.create_service(MoveToNamed, '~/move_to_named', self._named, callback_group=motion)
        self.create_service(MoveToJoints, '~/move_to_joints', self._joints, callback_group=motion)
        self.create_service(MoveToPose, '~/move_to_pose', self._pose, callback_group=motion)
        self.create_service(MoveRelative, '~/move_relative', self._relative, callback_group=motion)
        self.create_service(Gripper, '~/gripper', self._gripper, callback_group=motion)
        self.create_service(SceneObject, '~/scene', self._scene, callback_group=motion)
        self.create_service(PickPlace, '~/pick_place', self._pick_place, callback_group=motion)
        other = MutuallyExclusiveCallbackGroup()  # answered even while a motion runs
        self.create_service(GetArmState, '~/get_state', self._state, callback_group=other)
        self.create_service(Trigger, '~/stop', self._stop, callback_group=other)

    async def _run(self, what: str, action: Awaitable[object]) -> tuple[bool, str]:
        """Await a primitive and turn the outcome into (success, message)."""
        try:
            await action
        except (PrimitiveError, ValueError) as error:
            self.get_logger().error(f'{what}: {error}')
            return False, str(error)
        return True, f'{what}: done'

    async def _named(
        self, request: MoveToNamed.Request, response: MoveToNamed.Response
    ) -> MoveToNamed.Response:
        arm = self.robot.arm
        response.success, response.message = await self._run(
            f'move_to_named {request.name}',
            arm.move_to_named(request.name, velocity_scaling=_positive(request.velocity_scaling)),
        )
        return response

    async def _joints(
        self, request: MoveToJoints.Request, response: MoveToJoints.Response
    ) -> MoveToJoints.Response:
        if len(request.names) != len(request.positions):
            response.success, response.message = False, 'names and positions differ in length'
            return response
        joints = dict(zip(request.names, request.positions, strict=True))
        response.success, response.message = await self._run(
            'move_to_joints',
            self.robot.arm.move_to_joints(
                joints, velocity_scaling=_positive(request.velocity_scaling)
            ),
        )
        return response

    async def _pose(
        self, request: MoveToPose.Request, response: MoveToPose.Response
    ) -> MoveToPose.Response:
        response.success, response.message = await self._run(
            'move_to_pose',
            self.robot.arm.move_to_pose(
                request.pose,
                linear=request.linear,
                velocity_scaling=_positive(request.velocity_scaling),
            ),
        )
        return response

    async def _relative(
        self, request: MoveRelative.Request, response: MoveRelative.Response
    ) -> MoveRelative.Response:
        t, r = request.translation, request.rotation
        response.success, response.message = await self._run(
            'move_relative',
            self.robot.arm.move_relative(
                (t.x, t.y, t.z),
                (r.x, r.y, r.z),
                frame=request.frame or MoveRelative.Request.BASE,
                linear=request.linear,
                velocity_scaling=_positive(request.velocity_scaling),
            ),
        )
        return response

    async def _gripper(
        self, request: Gripper.Request, response: Gripper.Response
    ) -> Gripper.Response:
        gripper = self.robot.gripper
        command = request.command
        actions = {
            Gripper.Request.OPEN: gripper.open,
            Gripper.Request.CLOSE: gripper.close,
            Gripper.Request.HOME: gripper.home,
        }
        if command in actions:
            action = actions[command]()
        elif command == Gripper.Request.MOVE:
            action = gripper.move(request.width)
        elif command == Gripper.Request.GRASP:
            action = gripper.grasp(_positive(request.width), _positive(request.force))
        else:
            response.success, response.message = False, f'unknown gripper command "{command}"'
            return response
        response.success, response.message = await self._run(f'gripper {command}', action)
        return response

    async def _scene(
        self, request: SceneObject.Request, response: SceneObject.Response
    ) -> SceneObject.Response:
        scene = self.robot.scene
        op, name = request.operation, request.name
        shapes = {
            SceneObject.Request.ADD_BOX: 'box',
            SceneObject.Request.ADD_CYLINDER: 'cylinder',
            SceneObject.Request.ADD_SPHERE: 'sphere',
        }
        if op in shapes:
            action = scene.add(name, shapes[op], request.pose, list(request.dimensions))
        elif op == SceneObject.Request.REMOVE:
            action = scene.remove(name)
        elif op == SceneObject.Request.CLEAR:
            action = scene.clear()
        elif op == SceneObject.Request.ATTACH:
            action = scene.attach(name)
        elif op == SceneObject.Request.DETACH:
            action = scene.detach(name)
        else:
            response.success, response.message = False, f'unknown scene operation "{op}"'
            return response
        response.success, response.message = await self._run(f'scene {op} {name}', action)
        return response

    async def _pick_place(
        self, request: PickPlace.Request, response: PickPlace.Response
    ) -> PickPlace.Response:
        robot = self.robot
        approach, lift = _positive(request.approach), _positive(request.lift)
        if request.operation == PickPlace.Request.PICK:
            action = robot.pick(
                request.pose,
                request.object_name,
                approach,
                lift,
                _positive(request.width),
                _positive(request.force),
            )
        elif request.operation == PickPlace.Request.PLACE:
            action = robot.place(request.pose, request.object_name, approach, lift)
        else:
            response.success = False
            response.message = f'unknown operation "{request.operation}"'
            return response
        response.success, response.message = await self._run(request.operation, action)
        return response

    async def _state(
        self, request: GetArmState.Request, response: GetArmState.Response
    ) -> GetArmState.Response:
        arm = self.robot.arm
        config = self.robot.config
        response.arm, response.base_frame, response.ee_link = (
            config.name,
            config.base_frame,
            config.ee_link,
        )
        try:
            positions = arm.joint_positions()
            response.joint_state = JointState(
                name=list(positions), position=list(positions.values())
            )
            response.ee_pose = PoseStamped(pose=await arm.fk())
            response.ee_pose.header.frame_id = config.base_frame
            response.ee_pose.header.stamp = self.get_clock().now().to_msg()
        except PrimitiveError as error:
            response.success, response.message = False, str(error)
            return response
        response.success, response.message = True, ''
        return response

    def _stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.robot.arm.stop()
        response.success, response.message = True, 'stop sent'
        return response


def main(args: list[str] | None = None) -> None:
    """Run the primitives server."""
    # Ctrl-C raises KeyboardInterrupt here instead of rclpy shutting the context down
    # under the executor's threads.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    executor = MultiThreadedExecutor()
    try:
        node = PrimitivesNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt, ExternalShutdownException:
        pass
    finally:
        executor.shutdown()
        rclpy.try_shutdown()
