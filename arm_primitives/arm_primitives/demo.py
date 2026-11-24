"""
Run every primitive on the configured arm and check where the arm ends up.

ros2 run arm_primitives demo --ros-args -p arm:=fer   (or arm:=ur7e)

All targets are offsets from the arm's 'ready' pose, so the same sequence fits every
arm. Exits with status 1 if a primitive fails or a pose check is off.
"""

import sys

from geometry_msgs.msg import Pose

from arm_primitives.errors import PrimitiveError
from arm_primitives.geometry import make_pose, offset_pose, orientation_error, position_error
from arm_primitives.robot import Robot, run

POSITION_CHECK = 0.005  # m
ORIENTATION_CHECK = 0.02  # rad


def describe(pose: Pose) -> str:
    """Return a short text form of a pose."""
    p, q = pose.position, pose.orientation
    return (
        f'xyz=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) xyzw=({q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f})'
    )


async def check(robot: Robot, expected: Pose, what: str) -> None:
    """Raise PrimitiveError if the end-effector is not at the expected pose."""
    actual = await robot.arm.ee_pose()
    dp, dq = position_error(expected, actual), orientation_error(expected, actual)
    robot.node.get_logger().info(
        f'{what}: {describe(actual)} (off by {dp * 1000:.1f} mm, {dq:.3f} rad)'
    )
    if dp > POSITION_CHECK or dq > ORIENTATION_CHECK:
        raise PrimitiveError(f'{what}: ended {dp * 1000:.1f} mm / {dq:.3f} rad from the target')


async def exercise(robot: Robot) -> None:
    """Run the primitives in sequence; raises on the first failure."""
    arm, scene, log = robot.arm, robot.scene, robot.node.get_logger()
    log.info(f'demo on "{robot.config.name}"; named poses: {arm.named_poses()}')

    await arm.move_to_named('ready')
    ready = await arm.ee_pose()
    log.info(f'ready: {describe(ready)}')

    joints = arm.joint_positions()
    first = robot.config.joints[0]
    await arm.move_to_joints({first: joints[first] + 0.3})
    await arm.move_to_named('ready')
    await check(robot, ready, 'joint move and back')

    await arm.move_relative((0.0, 0.0, -0.05))
    await check(robot, offset_pose(ready, (0.0, 0.0, -0.05)), 'relative move down (base)')
    before = await arm.ee_pose()
    await arm.move_relative((0.03, 0.0, 0.0), frame='tool')
    await check(robot, offset_pose(before, (0.03, 0.0, 0.0), frame='tool'), 'relative move (tool)')
    before = await arm.ee_pose()
    await arm.move_relative(rotation=(0.0, 0.0, 0.3), frame='tool')
    await check(robot, offset_pose(before, rotation=(0.0, 0.0, 0.3), frame='tool'), 'tool twist')

    target = offset_pose(ready, (0.05, 0.1, -0.1))
    await arm.move_to_pose(target)
    await check(robot, target, 'free-space pose move')

    corner = await arm.ee_pose()
    square = [
        offset_pose(corner, (0.05, 0.0, 0.0)),
        offset_pose(corner, (0.05, 0.05, 0.0)),
        offset_pose(corner, (0.0, 0.05, 0.0)),
        corner,
    ]
    await arm.move_cartesian(square)
    await check(robot, corner, 'Cartesian square')

    plan = await arm.move_to_named('ready', execute=False)
    await arm.execute(plan)
    await check(robot, ready, 'plan then execute')

    solution = await arm.ik(target)
    reached = await arm.fk(solution)
    if position_error(target, reached) > POSITION_CHECK:
        raise PrimitiveError('IK/FK round trip does not match')
    log.info('IK/FK round trip ok')

    # Scene: a 4 cm cube on a 2 cm table, 25 cm below the ready pose
    p = ready.position
    x, y, z = p.x, p.y, p.z - 0.25
    grasp = arm.tool_down_pose(x, y, z)  # tool center on the cube center
    release = offset_pose(grasp, (0.0, 0.12, 0.0))
    await scene.clear()
    await scene.add_box('table', make_pose((x, y + 0.06, z - 0.03)), (0.3, 0.4, 0.02))
    await scene.add_box('cube', make_pose((x, y, z)), (0.04, 0.04, 0.04))
    if robot.gripper.available:
        await robot.gripper.home()
        await robot.pick(grasp, 'cube', width=0.04)
        await robot.place(release, 'cube')
        retreat = robot.config.approach_distance
        await check(robot, offset_pose(release, (0.0, 0.0, -retreat), frame='tool'), 'pick/place')
    else:
        log.info('no gripper: attach / carry / detach instead of pick and place')
        above = offset_pose(grasp, (0.0, 0.0, 0.1))
        await arm.move_to_pose(above)
        await arm.move_to_pose(grasp, linear=True, avoid_collisions=False)
        await scene.attach('cube')
        await arm.move_relative((0.0, 0.0, 0.1), avoid_collisions=False)
        await arm.move_to_pose(offset_pose(release, (0.0, 0.0, 0.1)))
        await arm.move_to_pose(release, linear=True, avoid_collisions=False)
        await scene.detach('cube')
        await arm.move_relative((0.0, 0.0, 0.1), avoid_collisions=False)
    await arm.move_to_named('ready')
    await scene.clear()
    log.info(f'DEMO PASSED on "{robot.config.name}"')


def main(args: list[str] | None = None) -> None:
    """Run the demo; exit 1 on failure."""
    try:
        run(exercise, node_name='arm_primitives_demo', args=args)
    except (PrimitiveError, ValueError) as error:
        print(f'DEMO FAILED: {error}', file=sys.stderr)
        sys.exit(1)
