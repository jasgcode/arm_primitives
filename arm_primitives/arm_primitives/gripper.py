"""
Gripper primitives behind one interface: open, close, move, grasp, home.

make_gripper() picks the implementation from GripperConfig.type, so code written
against Gripper works with the Franka Hand, any GripperCommand gripper, or no gripper.
"""

from abc import ABC, abstractmethod
from typing import override

from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node

from arm_primitives._ros import call_action
from arm_primitives.config import GripperConfig
from arm_primitives.errors import GripperError


class Gripper(ABC):
    """
    Gripper primitives shared by every gripper.

    Widths are the distance between the fingers in m, from config.min_width (closed)
    to config.max_width (open).
    """

    def __init__(self, config: GripperConfig) -> None:
        self.config = config

    @property
    def available(self) -> bool:
        """True if the arm has a gripper."""
        return True

    async def open(self) -> None:
        """Open the gripper fully."""
        await self.move(self.config.max_width)

    async def close(self) -> None:
        """Close the gripper fully (use grasp() to hold an object)."""
        await self.move(self.config.min_width)

    async def home(self) -> None:
        """Calibrate the gripper if it supports it, else open it."""
        await self.open()

    @abstractmethod
    async def move(self, width: float, speed: float | None = None) -> None:
        """
        Move the fingers to a width without expecting to hold anything.

        :param width: target width in m
        :param speed: finger speed in m/s; None = config default
        :raises GripperError: if the command fails
        """

    @abstractmethod
    async def grasp(self, width: float | None = None, force: float | None = None) -> None:
        """
        Close on an object and hold it.

        :param width: expected object width in m; None = close until contact
        :param force: grasp force in N; None = config default
        :raises GripperError: if nothing is grasped or the command fails
        """

    def _check_width(self, width: float) -> float:
        """Return width if the gripper can reach it, else raise GripperError."""
        low, high = self.config.min_width, self.config.max_width
        if not low - 1e-9 <= width <= high + 1e-9:
            raise GripperError(f'width {width:.4f} m is outside [{low}, {high}]')
        return min(max(width, low), high)


class NoGripper(Gripper):
    """Stand-in for an arm without a gripper: every command raises GripperError."""

    @property
    @override
    def available(self) -> bool:
        """False: this arm has no gripper."""
        return False

    def _missing(self) -> GripperError:
        return GripperError('this arm has no gripper (gripper type "none" in its config)')

    @override
    async def open(self) -> None:
        """Raise GripperError."""
        raise self._missing()

    @override
    async def close(self) -> None:
        """Raise GripperError."""
        raise self._missing()

    @override
    async def home(self) -> None:
        """Raise GripperError."""
        raise self._missing()

    @override
    async def move(self, width: float, speed: float | None = None) -> None:
        """Raise GripperError."""
        raise self._missing()

    @override
    async def grasp(self, width: float | None = None, force: float | None = None) -> None:
        """Raise GripperError."""
        raise self._missing()


class FrankaGripper(Gripper):
    """
    Franka Hand through the franka_gripper node.

    Action Clients
    --------------
    <namespace>/move : franka_msgs/action/Move
    <namespace>/grasp : franka_msgs/action/Grasp
    <namespace>/homing : franka_msgs/action/Homing
    """

    def __init__(self, node: Node, config: GripperConfig, callback_group: CallbackGroup) -> None:
        super().__init__(config)
        # Imported here so arms without a Franka Hand do not need franka_msgs installed
        from franka_msgs.action import Grasp, Homing, Move

        self._goals = (Move.Goal, Grasp.Goal, Homing.Goal)
        ns = config.namespace
        self._move = ActionClient(node, Move, f'{ns}/move', callback_group=callback_group)
        self._grasp = ActionClient(node, Grasp, f'{ns}/grasp', callback_group=callback_group)
        self._homing = ActionClient(node, Homing, f'{ns}/homing', callback_group=callback_group)

    @override
    async def move(self, width: float, speed: float | None = None) -> None:
        """Move the fingers to a width (see Gripper.move)."""
        goal = self._goals[0](width=self._check_width(width), speed=speed or self.config.speed)
        result = await call_action(self._move, goal, f'{self.config.namespace}/move')
        if not result.success:
            raise GripperError(f'gripper move failed: {result.error}')

    @override
    async def grasp(self, width: float | None = None, force: float | None = None) -> None:
        """Close on an object and hold it (see Gripper.grasp)."""
        goal = self._goals[1](speed=self.config.speed, force=force or self.config.force)
        if width is None:
            # Unknown object width: accept anything between closed and fully open
            goal.width = self.config.min_width
            goal.epsilon.inner = self.config.epsilon
            goal.epsilon.outer = self.config.max_width
        else:
            goal.width = self._check_width(width)
            goal.epsilon.inner = self.config.epsilon
            goal.epsilon.outer = self.config.epsilon
        result = await call_action(self._grasp, goal, f'{self.config.namespace}/grasp')
        if not result.success:
            raise GripperError(f'gripper grasp failed: {result.error}')

    @override
    async def home(self) -> None:
        """Calibrate the finger width range, which leaves the gripper open."""
        result = await call_action(
            self._homing, self._goals[2](), f'{self.config.namespace}/homing'
        )
        if not result.success:
            raise GripperError(f'gripper homing failed: {result.error}')


class GripperCommandGripper(Gripper):
    """
    Any gripper with a control_msgs/action/GripperCommand server (Robotiq, ros2_control).

    A width w maps linearly to the command position: min_width -> closed_position,
    max_width -> open_position.

    Action Clients
    --------------
    <namespace> : control_msgs/action/GripperCommand
    """

    def __init__(self, node: Node, config: GripperConfig, callback_group: CallbackGroup) -> None:
        super().__init__(config)
        self._client = ActionClient(
            node, GripperCommand, config.namespace, callback_group=callback_group
        )

    def position(self, width: float) -> float:
        """Return the command position for a finger width in m."""
        c = self.config
        fraction = (self._check_width(width) - c.min_width) / (c.max_width - c.min_width)
        return c.closed_position + fraction * (c.open_position - c.closed_position)

    async def _command(self, width: float, max_effort: float) -> GripperCommand.Result:
        goal = GripperCommand.Goal()
        goal.command.position = self.position(width)
        goal.command.max_effort = max_effort
        return await call_action(self._client, goal, self.config.namespace)

    @override
    async def move(self, width: float, speed: float | None = None) -> None:
        """Move the fingers to a width (see Gripper.move); speed is set by the server."""
        result = await self._command(width, 0.0)
        if not result.reached_goal:
            raise GripperError(f'gripper stopped at position {result.position:.4f}')

    @override
    async def grasp(self, width: float | None = None, force: float | None = None) -> None:
        """Close on an object and hold it (see Gripper.grasp)."""
        target = self.config.min_width if width is None else width
        result = await self._command(target, force or self.config.force)
        if not (result.stalled or result.reached_goal):
            raise GripperError('gripper grasp failed: neither stalled nor reached the goal')


def make_gripper(node: Node, config: GripperConfig, callback_group: CallbackGroup) -> Gripper:
    """
    Create the Gripper implementation config.type asks for.

    :param node: node the action clients belong to
    :param config: the arm's gripper config
    :param callback_group: callback group for the action clients
    :return: a Gripper
    """
    if config.type == 'franka':
        return FrankaGripper(node, config, callback_group)
    if config.type == 'gripper_command':
        return GripperCommandGripper(node, config, callback_group)
    return NoGripper(config)
