#!/usr/bin/env python3
#
"""
A common utility module for Allegro Hand motion scripts.

This module provides shared functions, such as activating controllers,
to avoid code duplication across different motion scripts.
"""
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers, SwitchController


def activate_controller(node: Node, controller_to_activate: str) -> bool:
    """
    Activates a specified controller via the controller manager if it's not already active.

    This function first checks the current state of all available controllers.
    If the target controller is already 'active', it returns True immediately.
    Otherwise, it sends a request to the '/controller_manager/switch_controller'
    service to activate the desired controller.

    Args:
        node: The ROS 2 node instance from which to create service clients.
        controller_to_activate: The name of the controller to be activated.

    Returns:
        True if the controller is or becomes active, False otherwise.
    """
    # First, check the current state of the controller to avoid unnecessary service calls.
    list_controllers_client = node.create_client(
        ListControllers, "/controller_manager/list_controllers"
    )
    if not list_controllers_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(
            "Could not connect to /controller_manager/list_controllers service."
        )
        return False

    list_req = ListControllers.Request()
    future = list_controllers_client.call_async(list_req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if future.result():
        for controller in future.result().controller:
            if controller.name == controller_to_activate:
                if controller.state == "active":
                    node.get_logger().info(
                        f"Controller '{controller_to_activate}' is already active."
                    )
                    return True
                # Controller found, but not active. Break and proceed to activate.
                break
    else:
        # This is not a fatal error; we can still attempt to activate the controller.
        node.get_logger().warn(
            "Failed to get controller list. Proceeding with activation attempt."
        )

    # If the controller is not already active, request to switch to it.
    switch_controller_client = node.create_client(
        SwitchController, "/controller_manager/switch_controller"
    )

    if not switch_controller_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(
            "Could not connect to /controller_manager/switch_controller service."
        )
        return False

    req = SwitchController.Request()
    req.activate_controllers = [controller_to_activate]
    # STRICT ensures that the activation will fail if any other controller
    # claims the same resources.
    req.strictness = SwitchController.Request.STRICT

    node.get_logger().info(f"Activating controller: '{controller_to_activate}'...")

    future = switch_controller_client.call_async(req)

    # Block until the service call is complete or times out.
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if future.result() is not None:
        if future.result().ok:
            node.get_logger().info(
                f"Successfully activated controller '{controller_to_activate}'"
            )
            return True
        else:
            node.get_logger().error(
                f"Failed to activate controller '{controller_to_activate}'"
            )
            return False
    else:
        node.get_logger().error(
            f"Service call to activate controller '{controller_to_activate}' timed out."
        )
        return False
