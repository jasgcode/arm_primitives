"""Exceptions raised by the primitives."""


class PrimitiveError(RuntimeError):
    """A primitive could not be carried out (planning, execution, or a missing server)."""


class GripperError(PrimitiveError):
    """The gripper is missing or a gripper command failed."""
