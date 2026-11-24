"""Scene primitives: collision objects and attached objects in the MoveIt planning scene."""

from collections.abc import Mapping, Sequence

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
    PlanningSceneWorld,
)
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.callback_groups import CallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from arm_primitives._ros import call_service
from arm_primitives.config import ArmConfig
from arm_primitives.errors import PrimitiveError
from arm_primitives.geometry import make_pose

SHAPES = {
    'box': (SolidPrimitive.BOX, 3),  # dimensions: x, y, z
    'cylinder': (SolidPrimitive.CYLINDER, 2),  # dimensions: height, radius
    'sphere': (SolidPrimitive.SPHERE, 1),  # dimensions: radius
}


class Scene:
    """
    Add, remove, attach and detach collision objects.

    Poses given as Pose are in config.base_frame.

    Service Clients
    ---------------
    apply_planning_scene : moveit_msgs/srv/ApplyPlanningScene
    """

    def __init__(
        self, node: Node, config: ArmConfig, callback_group: CallbackGroup | None = None
    ) -> None:
        """
        Create the planning scene client on the user's node.

        :param node: node shared by all components
        :param config: the arm's config (frames, attach link, touch links)
        :param callback_group: group for the client; None = a new ReentrantCallbackGroup
        """
        self.node = node
        self.config = config
        self._client = node.create_client(
            ApplyPlanningScene,
            'apply_planning_scene',
            callback_group=callback_group or ReentrantCallbackGroup(),
        )

    async def apply(self, scene: PlanningScene) -> None:
        """
        Apply a planning scene diff.

        :param scene: moveit_msgs/msg/PlanningScene with the changes
        :raises PrimitiveError: if MoveIt rejects the change
        """
        scene.is_diff = True
        scene.robot_state.is_diff = True
        response = await call_service(self._client, ApplyPlanningScene.Request(scene=scene))
        if not response.success:
            raise PrimitiveError('MoveIt rejected the planning scene change')

    async def add(
        self, name: str, shape: str, pose: Pose | PoseStamped, dimensions: Sequence[float]
    ) -> None:
        """
        Add (or replace) a collision object.

        :param name: object id
        :param shape: 'box', 'cylinder' or 'sphere'
        :param pose: pose of the object's center (Pose = in config.base_frame)
        :param dimensions: box [x, y, z]; cylinder [height, radius]; sphere [radius] (m)
        """
        if shape not in SHAPES:
            raise PrimitiveError(f'shape must be one of {sorted(SHAPES)}, got "{shape}"')
        kind, count = SHAPES[shape]
        if len(dimensions) != count or any(d <= 0.0 for d in dimensions):
            raise PrimitiveError(f'a {shape} needs {count} positive dimensions')
        obj = CollisionObject(id=name, operation=CollisionObject.ADD)
        if isinstance(pose, PoseStamped) and pose.header.frame_id:
            obj.header.frame_id = pose.header.frame_id
        else:
            obj.header.frame_id = self.config.base_frame
        obj.pose = pose.pose if isinstance(pose, PoseStamped) else pose
        obj.primitives = [SolidPrimitive(type=kind, dimensions=[float(d) for d in dimensions])]
        obj.primitive_poses = [Pose()]  # shape centered on obj.pose
        await self.apply(PlanningScene(world=PlanningSceneWorld(collision_objects=[obj])))

    async def add_box(self, name: str, pose: Pose | PoseStamped, size: Sequence[float]) -> None:
        """Add a box with edge lengths size = (x, y, z) in m."""
        await self.add(name, 'box', pose, size)

    async def add_cylinder(
        self, name: str, pose: Pose | PoseStamped, height: float, radius: float
    ) -> None:
        """Add a cylinder along the pose's z axis."""
        await self.add(name, 'cylinder', pose, (height, radius))

    async def add_sphere(self, name: str, pose: Pose | PoseStamped, radius: float) -> None:
        """Add a sphere."""
        await self.add(name, 'sphere', pose, (radius,))

    async def remove(self, name: str) -> None:
        """Remove a collision object from the world (detach it first if it is held)."""
        obj = CollisionObject(id=name, operation=CollisionObject.REMOVE)
        obj.header.frame_id = self.config.base_frame
        scene = PlanningScene()
        scene.world.collision_objects = [obj]
        await self.apply(scene)

    async def clear(self) -> None:
        """Remove every collision object from the world."""
        await self.remove('')  # MoveIt treats REMOVE with an empty id as "all objects"

    async def attach(
        self, name: str, link: str | None = None, touch_links: Sequence[str] | None = None
    ) -> None:
        """
        Attach a world object to the arm so it moves (and is collision checked) with it.

        :param name: id of an object already in the world
        :param link: link to attach to; None = config.attach_link (or ee_link)
        :param touch_links: links allowed to touch it; None = config.touch_links
        """
        attached = AttachedCollisionObject(link_name=link or self.config.attach)
        attached.object.id = name
        attached.object.operation = CollisionObject.ADD
        attached.touch_links = list(
            self.config.touch_links if touch_links is None else touch_links
        )
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [attached]
        await self.apply(scene)

    async def detach(self, name: str, link: str | None = None) -> None:
        """
        Detach a held object; it stays in the world where it was released.

        :param name: id of the attached object
        :param link: link it is attached to; None = config.attach_link (or ee_link)
        """
        attached = AttachedCollisionObject(link_name=link or self.config.attach)
        attached.object.id = name
        attached.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [attached]
        await self.apply(scene)

    async def load(self, objects: Sequence[Mapping[str, object]]) -> None:
        """
        Add several objects, e.g. from a YAML list.

        Each entry: {name: str, shape: box|cylinder|sphere, position: [x, y, z],
        orientation: [x, y, z, w] (optional), dimensions: [...], frame_id: str (optional)}.

        :param objects: the object descriptions
        """
        for entry in objects:
            name, shape = entry.get('name'), entry.get('shape', 'box')
            position, dimensions = entry.get('position'), entry.get('dimensions')
            orientation = entry.get('orientation', [0.0, 0.0, 0.0, 1.0])
            if not isinstance(name, str) or not isinstance(shape, str):
                raise PrimitiveError(f'scene object needs a "name" and "shape": {entry}')
            if not (
                isinstance(position, list)
                and isinstance(orientation, list)
                and isinstance(dimensions, list)
            ):
                raise PrimitiveError(f'scene object "{name}" needs position and dimensions lists')
            pose = PoseStamped(
                pose=make_pose([float(v) for v in position], [float(v) for v in orientation])
            )
            frame = entry.get('frame_id', '')
            pose.header.frame_id = frame if isinstance(frame, str) else ''
            await self.add(name, shape, pose, [float(v) for v in dimensions])
