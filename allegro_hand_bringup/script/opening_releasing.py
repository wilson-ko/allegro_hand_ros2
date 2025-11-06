#!/usr/bin/env python3
#
"""
A ROS 2 node to command the Allegro Hand to repeatedly open and close its fist.

This node uses linear interpolation to create a smooth transition between
the open and closed hand poses. It activates the specified controller upon
startup and can be configured via a ROS parameter.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
from common import activate_controller

class MotionGeneratorV4:
    """
    Generates joint commands for an opening and closing motion for the Allegro Hand V4.
    """

    def __init__(self):
        # The command array is a 16-element list corresponding to the following joint order:
        # - Joints 0-3:   Thumb (joint00, joint01, joint02, joint03)
        # - Joints 4-7:   Index (joint10, joint11, joint12, joint13)
        # - Joints 8-11:  Middle (joint20, joint21, joint22, joint23)
        # - Joints 12-15: Ring (joint30, joint31, joint32, joint33)

        # Define the target joint positions for the open and closed hand poses.
        self.open_position = np.array([0.0] * 16)

        # Each row corresponds to a finger: Thumb, Index, Middle, Ring.
        # The .flatten() method converts the 2D array into a 1D array of 16 elements.
        self.close_position = np.array(
            [
                [1.2, 1.5, 1.5, 1.5],  # Thumb
                [0.0, 1.45, 1.5, 0.0],  # Index Finger
                [0.0, 1.45, 1.5, 0.0],  # Middle Finger
                [0.0, 1.45, 1.5, 0.0],  # Ring Finger
            ]
        ).flatten()

        # The base position to return to on shutdown is the open position.
        self.base_position = self.open_position

    def get_command(self, start_pos, end_pos, ratio):
        """Calculates and returns the interpolated joint command."""
        return start_pos + (end_pos - start_pos) * ratio


class MotionGeneratorPlexus:
    """
    Generates joint commands for an opening and closing motion,
    specifically tailored for the Allegro Hand Plexus.
    """

    def __init__(self):
        # Define the target joint positions for the open and closed hand poses.
        # The first thumb joint's motion is centered at 0.15 rad with an amplitude of 0.5 rad.
        # This results in a range of [-0.1, 0.4] rad.
        # open_position[0] = 0.15 (center) - 0.25 (half_amplitude) = -0.1
        # The second thumb joint's motion is centered at 0.6 rad with an amplitude of 0.6 rad.
        # open_position[1] = 0.6 (center) - 0.3 (half_amplitude) = 0.3.
        # The third thumb joint's motion is centered at 0.6 rad with an amplitude of 0.6 rad.
        # open_position[2] = 0.6 (center) - 0.2 (half_amplitude) = 0.4.
        # The first joints of Index, Middle, and Ring fingers will move by +/- 20 degrees.
        # -20 degrees is approx -0.35 rad.
        self.open_position = np.array(
            [-0.1, 0.3, 0.4, 0.0] + [-0.35, 0.0, 0.0, 0.0] * 3
        )

        # The close position is slightly adjusted for the Plexus hand to avoid
        # potential collisions, especially for the thumb.
        # close_position[0] = 0.15 (center) + 0.25 (half_amplitude) = 0.4
        # +20 degrees is approx +0.35 rad.
        self.close_position = np.array(
            [
                [0.4, 0.9, 0.8, 1.2],  # Thumb (amplitudes adjusted)
                [0.35, 1.45, 1.5, 0.0],  # Index Finger
                [0.35, 1.45, 1.5, 0.0],  # Middle Finger
                [0.35, 1.45, 1.5, 0.0],  # Ring Finger
            ]
        ).flatten()

        self.output_filter = np.array(
            [
                [True, True, True, True],  # Thumb
                [True, True, True, True],  # Index Finger
                [True, True, True, True],  # Middle Finger
                [True, True, True, True],  # Ring Finger
            ]
        ).flatten()

        # The base position to return to on shutdown is the open position.
        self.base_position = self.open_position

    def get_command(self, start_pos, end_pos, ratio):
        """Calculates and returns the interpolated joint command."""
        command = start_pos + (end_pos - start_pos) * ratio
        return command * self.output_filter


def motion_generator_factory(device_name):
    """
    Factory function to create the appropriate motion generator based on device name.
    """
    if device_name == "v4":
        return MotionGeneratorV4()
    elif device_name == "plexus":
        return MotionGeneratorPlexus()
    else:
        raise ValueError(
            f"Unknown device name: {device_name}. Supported devices are 'v4' and 'plexus'."
        )


class AllegroHandMotionController(Node):
    """
    Controls the Allegro Hand to perform an opening and closing motion.

    This class initializes a ROS 2 node, activates the target controller,
    and then periodically publishes interpolated joint commands to create the
    fist-closing and opening motion.
    """

    def __init__(self):
        super().__init__("opening_releasing_node")
        self.cmd_publisher_ = None

        # Declare and get the controller name parameter. This allows the node
        # to be flexible and work with different controller configurations.
        self.declare_parameter("controller_name", "allegro_hand_position_controller")
        self.declare_parameter("device_name", "v4")

        self.device_name = (
            self.get_parameter("device_name").get_parameter_value().string_value
        )
        self.controller_name = (
            self.get_parameter("controller_name").get_parameter_value().string_value
        )
        pos_topic_name = f"/{self.controller_name}/commands"

        # Activate the target controller before starting any motion.
        # If activation fails, the node will log an error and not proceed.
        if not activate_controller(self, self.controller_name):
            self.get_logger().error(
                f"Failed to activate controller '{self.controller_name}'. "
                "Node will not perform any motion."
            )
            return

        # Create the publisher only after the controller is successfully activated.
        self.cmd_publisher_ = self.create_publisher(Float64MultiArray, pos_topic_name, 10)

        # Parameters for smooth motion generation.
        self.motion_duration = (
            1.0  # seconds for one direction (e.g., from open to close).
        )
        self.update_rate = 50  # Hz
        self.timer_period = 1.0 / self.update_rate
        self.total_steps = int(self.motion_duration * self.update_rate)

        self.timer = self.create_timer(self.timer_period, self.motion_callback)
        self.get_logger().info(
            f"Allegro Hand Motion Controller Node Started (Update rate: {self.update_rate}Hz)"
        )
        self.get_logger().info(f"Publishing position commands to: {pos_topic_name}")

        # State variables to track the motion progress.
        self.is_closing = True  # True if the hand is closing, False if opening.
        self.step = 0  # Current step in the interpolation.

        # Instantiate the motion generator using the factory
        try:
            self.motion_generator = motion_generator_factory(self.device_name)
            self.get_logger().info(
                f"Using motion generator for device: '{self.device_name}'"
            )
        except ValueError as e:
            self.get_logger().error(str(e))
            return

    def motion_callback(self):
        """
        Called periodically by the timer to calculate and publish new joint commands.
        """
        pos_msg = Float64MultiArray()

        # Determine the start and end positions for the current motion direction.
        if self.is_closing:
            start_pos = self.motion_generator.open_position
            end_pos = self.motion_generator.close_position
        else:  # is_opening
            start_pos = self.motion_generator.close_position
            end_pos = self.motion_generator.open_position

        # Calculate the interpolated position for the current step.
        ratio = float(self.step) / self.total_steps
        current_position = self.motion_generator.get_command(start_pos, end_pos, ratio)
        pos_msg.data = current_position.tolist()

        self.cmd_publisher_.publish(pos_msg)



        # Update the step and, if the motion is complete, switch direction.
        self.step += 1
        if self.step > self.total_steps:
            self.step = 0
            self.is_closing = not self.is_closing
            # if self.is_closing:
            #     self.get_logger().info("Starting to close hand...")
            # else:
            #     self.get_logger().info("Starting to open hand...")


def main(args=None):
    """The main entry point for the script."""
    rclpy.init(args=args)
    motion_controller = AllegroHandMotionController()
    try:
        # Spin the node to process callbacks, services, etc.
        rclpy.spin(motion_controller)
    except KeyboardInterrupt:
        # The user pressed Ctrl-C.
        pass
    finally:
        # This block executes on shutdown (including Ctrl-C).
        # Return the hand to a safe, open position.
        # These checks are important in case the node failed to initialize publishers.
        if motion_controller.cmd_publisher_:
            pos_msg = Float64MultiArray()
            pos_msg.data = motion_controller.motion_generator.base_position.tolist()
            motion_controller.cmd_publisher_.publish(pos_msg)
            motion_controller.get_logger().info("Returning to open position.")

        motion_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
