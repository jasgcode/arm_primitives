"""
Pose math for the primitives: quaternions, offsets, and pose errors.

Plain Python on top of geometry_msgs, so it works the same for every arm.
Quaternions are (x, y, z, w) tuples or geometry_msgs/Quaternion.
"""

from collections.abc import Sequence
import math

from geometry_msgs.msg import Point, Pose, Quaternion

type Vec3 = tuple[float, float, float]
type Quat = tuple[float, float, float, float]

BASE = 'base'  # offsets along the arm base frame axes
TOOL = 'tool'  # offsets along the end-effector axes


def as_quat(q: Quaternion | Sequence[float]) -> Quat:
    """
    Return a quaternion as an (x, y, z, w) tuple.

    :param q: geometry_msgs/Quaternion or a sequence of 4 numbers (x, y, z, w)
    :return: (x, y, z, w)
    """
    if isinstance(q, Quaternion):
        return (q.x, q.y, q.z, q.w)
    if len(q) != 4:
        raise ValueError(f'a quaternion needs 4 values (x, y, z, w), got {len(q)}')
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def as_vec3(v: Point | Sequence[float]) -> Vec3:
    """
    Return a point or vector as an (x, y, z) tuple.

    :param v: geometry_msgs/Point or a sequence of 3 numbers
    :return: (x, y, z)
    """
    if isinstance(v, Point):
        return (v.x, v.y, v.z)
    if len(v) != 3:
        raise ValueError(f'a vector needs 3 values (x, y, z), got {len(v)}')
    return (float(v[0]), float(v[1]), float(v[2]))


def to_quaternion(q: Quat) -> Quaternion:
    """Return an (x, y, z, w) tuple as a geometry_msgs/Quaternion."""
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def normalize(q: Quat) -> Quat:
    """Return the unit quaternion pointing the same way as q."""
    norm = math.sqrt(sum(c * c for c in q))
    if norm < 1e-12:
        raise ValueError('cannot normalize a zero quaternion')
    return (q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm)


def multiply(a: Quat, b: Quat) -> Quat:
    """Return the Hamilton product a * b (rotate by b, then by a)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def conjugate(q: Quat) -> Quat:
    """Return the inverse rotation of the unit quaternion q."""
    return (-q[0], -q[1], -q[2], q[3])


def rotate(q: Quat, v: Vec3) -> Vec3:
    """Return the vector v rotated by the unit quaternion q."""
    x, y, z, _ = multiply(multiply(q, (v[0], v[1], v[2], 0.0)), conjugate(q))
    return (x, y, z)


def from_rpy(roll: float, pitch: float, yaw: float) -> Quat:
    """
    Return the quaternion for fixed-axis roll, pitch, yaw (rad).

    Same convention as tf2: rotate about x by roll, then y by pitch, then z by yaw.
    """
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def to_rpy(q: Quat) -> Vec3:
    """Return the fixed-axis roll, pitch, yaw (rad) of the unit quaternion q."""
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (roll, pitch, yaw)


def make_pose(
    position: Point | Sequence[float],
    orientation: Quaternion | Sequence[float] = (0.0, 0.0, 0.0, 1.0),
) -> Pose:
    """
    Build a geometry_msgs/Pose.

    :param position: (x, y, z) in m
    :param orientation: (x, y, z, w); normalized before use
    :return: geometry_msgs/Pose
    """
    x, y, z = as_vec3(position)
    return Pose(
        position=Point(x=x, y=y, z=z), orientation=to_quaternion(normalize(as_quat(orientation)))
    )


def offset_pose(
    pose: Pose,
    translation: Sequence[float] = (0.0, 0.0, 0.0),
    rotation: Sequence[float] = (0.0, 0.0, 0.0),
    frame: str = BASE,
) -> Pose:
    """
    Return the pose moved by a translation and a roll/pitch/yaw rotation.

    The rotation turns the pose about its own position, so the position only changes
    by the translation.

    :param pose: starting pose
    :param translation: (dx, dy, dz) in m
    :param rotation: (roll, pitch, yaw) in rad
    :param frame: BASE = along/about the axes the pose is expressed in,
        TOOL = along/about the pose's own axes
    :return: the new geometry_msgs/Pose
    """
    if frame not in (BASE, TOOL):
        raise ValueError(f'frame must be "{BASE}" or "{TOOL}", got "{frame}"')
    q = normalize(as_quat(pose.orientation))
    t = as_vec3(translation)
    roll, pitch, yaw = as_vec3(rotation)
    dq = from_rpy(roll, pitch, yaw)
    if frame == TOOL:
        t = rotate(q, t)
        q_new = multiply(q, dq)
    else:
        q_new = multiply(dq, q)
    p = pose.position
    return make_pose((p.x + t[0], p.y + t[1], p.z + t[2]), q_new)


def position_error(a: Pose | Point, b: Pose | Point) -> float:
    """Return the distance in m between two positions."""
    pa = a.position if isinstance(a, Pose) else a
    pb = b.position if isinstance(b, Pose) else b
    return math.dist(as_vec3(pa), as_vec3(pb))


def orientation_error(a: Pose | Quaternion, b: Pose | Quaternion) -> float:
    """Return the angle in rad of the rotation between two orientations, in [0, pi]."""
    qa = normalize(as_quat(a.orientation if isinstance(a, Pose) else a))
    qb = normalize(as_quat(b.orientation if isinstance(b, Pose) else b))
    dot = abs(sum(ca * cb for ca, cb in zip(qa, qb, strict=True)))
    return 2.0 * math.acos(min(1.0, dot))
