"""Utility functions shared across robin_calibration modules."""

import time

from std_srvs.srv import SetBool


def wait_for_future(node, future, timeout_sec: float = 5.0):
    """Wait for a future to complete without creating a conflicting executor.

    With MultiThreadedExecutor, service responses arrive on executor threads.
    We just poll future.done() instead of rclpy.spin_until_future_complete,
    which would create a temporary SingleThreadedExecutor and deadlock.

    Args:
        node: ROS2 node (used for logging only)
        future: rclpy future to wait on
        timeout_sec: Maximum seconds to wait

    Returns:
        The future result, or None on timeout.
    """
    start = time.monotonic()
    while not future.done():
        if time.monotonic() - start > timeout_sec:
            node.get_logger().warn(
                f"Future timed out after {timeout_sec:.1f}s")
            return None
        time.sleep(0.01)  # 10ms poll, executor threads handle callbacks
    return future.result()


def set_wago_signal_direct(node, service_name: str, value: bool):
    """Directly call a WAGO SetBool service (convenience helper for cleanup).

    Creates a temporary client, sends the request, and destroys the client.

    Args:
        node: ROS2 node to create the client on
        service_name: Full service name, e.g. '/wago/in/touch_sensing'
        value: Boolean value to send
    """
    client = node.create_client(SetBool, service_name)
    if client.wait_for_service(timeout_sec=1.0):
        req = SetBool.Request()
        req.data = value
        future = client.call_async(req)
        wait_for_future(node, future, timeout_sec=2.0)
    client.destroy()
