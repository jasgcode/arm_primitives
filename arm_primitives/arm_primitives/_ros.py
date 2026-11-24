"""Awaitable service and action calls that raise PrimitiveError instead of hanging."""

from typing import Any

from rclpy.action import ActionClient
from rclpy.client import Client
from rosidl_pycommon.interface_base_classes import BaseImpl, BaseMessage

from arm_primitives.errors import PrimitiveError

SERVER_TIMEOUT = 5.0  # s to wait for a service or action server to appear


async def call_service[RequestT: BaseMessage, ResponseT: BaseMessage](
    client: Client[RequestT, ResponseT],
    request: RequestT,
    timeout: float = SERVER_TIMEOUT,
) -> ResponseT:
    """
    Call a service and wait for the response.

    :param client: the service client
    :param request: the request message
    :param timeout: seconds to wait for the service to appear
    :return: the response message
    :raises PrimitiveError: if the service is unavailable or returns no response
    """
    if not client.wait_for_service(timeout_sec=timeout):
        raise PrimitiveError(f'service "{client.srv_name}" is unavailable')
    response = await client.call_async(request)
    if response is None:
        raise PrimitiveError(f'service "{client.srv_name}" returned no response')
    return response


async def call_action[
    GoalT: BaseMessage,
    ResultT: BaseMessage,
    FeedbackT: BaseMessage,
    ImplT: BaseImpl[Any, Any, Any],  # same bound as rclpy's ActionClient
](
    client: ActionClient[GoalT, ResultT, FeedbackT, ImplT],
    goal: GoalT,
    name: str,
    timeout: float = SERVER_TIMEOUT,
) -> ResultT:
    """
    Send an action goal and wait for its result.

    :param client: the action client
    :param goal: the goal message
    :param name: action name, for error messages
    :param timeout: seconds to wait for the action server to appear
    :return: the result message
    :raises PrimitiveError: if the server is unavailable, rejects the goal, or sends no result
    """
    if not client.wait_for_server(timeout_sec=timeout):
        raise PrimitiveError(f'action server "{name}" is unavailable')
    handle = await client.send_goal_async(goal)
    if handle is None or not handle.accepted:
        raise PrimitiveError(f'action server "{name}" rejected the goal')
    response = await handle.get_result_async()
    if response is None:
        raise PrimitiveError(f'action server "{name}" returned no result')
    return response.result
