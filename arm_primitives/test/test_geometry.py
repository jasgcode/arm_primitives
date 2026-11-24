"""Unit tests for the pose math (no ROS graph needed)."""

import math

from arm_primitives.geometry import (
    conjugate,
    from_rpy,
    make_pose,
    multiply,
    offset_pose,
    orientation_error,
    position_error,
    rotate,
    to_rpy,
)
import pytest

DOWN = (1.0, 0.0, 0.0, 0.0)  # rotation of pi about x: tool z axis points down


def close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b, strict=True))


def test_rpy_round_trip():
    for rpy in [(0.1, -0.2, 0.3), (1.0, 0.5, -2.0), (0.0, 0.0, math.pi / 2)]:
        assert close(to_rpy(from_rpy(*rpy)), rpy)


def test_rotate_about_z():
    q = from_rpy(0.0, 0.0, math.pi / 2)
    assert close(rotate(q, (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0))


def test_inverse():
    q = from_rpy(0.3, -0.4, 1.2)
    assert close(multiply(q, conjugate(q)), (0.0, 0.0, 0.0, 1.0))


def test_base_offset_only_moves_position():
    pose = make_pose((0.4, 0.0, 0.3), DOWN)
    moved = offset_pose(pose, (0.0, 0.0, -0.1))
    assert close((moved.position.x, moved.position.y, moved.position.z), (0.4, 0.0, 0.2))
    assert orientation_error(pose, moved) < 1e-9


def test_tool_offset_follows_tool_axes():
    # With the tool pointing down, +z of the tool is -z of the base
    pose = make_pose((0.4, 0.0, 0.3), DOWN)
    moved = offset_pose(pose, (0.0, 0.0, 0.1), frame='tool')
    assert position_error(moved, make_pose((0.4, 0.0, 0.2))) < 1e-9


def test_tool_rotation_keeps_position():
    pose = make_pose((0.4, 0.1, 0.3), DOWN)
    turned = offset_pose(pose, rotation=(0.0, 0.0, 0.5), frame='tool')
    assert position_error(pose, turned) < 1e-9
    assert abs(orientation_error(pose, turned) - 0.5) < 1e-9


def test_orientation_error_ignores_sign():
    q = from_rpy(0.2, 0.1, -0.3)
    a = make_pose((0.0, 0.0, 0.0), q)
    b = make_pose((0.0, 0.0, 0.0), tuple(-c for c in q))
    assert orientation_error(a, b) < 1e-6


def test_bad_frame():
    with pytest.raises(ValueError):
        offset_pose(make_pose((0.0, 0.0, 0.0)), frame='world')
