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
        # 각 관절의 움직임 범위 설정 (0도 ~ 30도)
        thirty_degrees_in_rad = 30.0 * np.pi / 180.0
        self.open_position = np.array([0.0] * 16)
        self.close_position = np.array([thirty_degrees_in_rad] * 16)

        # 움직임의 속도를 조절하는 주파수 (rad/s)
        self.frequency = 4.0

        # 각 관절에 위상차를 적용하여 순차적으로 움직이게 합니다.
        # 16개 관절에 걸쳐 2*PI 만큼의 위상차를 균등하게 분배합니다.
        self.phase_offsets = np.linspace(0, 2 * np.pi, 16, endpoint=False)

        self.output_filter = np.array(
            [
                [False, False, False, False],  # Thumb
                [False, True, False, False],  # Index Finger
                [False, False, False, False],  # Middle Finger
                [False, False, False, False],  # Ring Finger
            ]
        ).flatten()

        # The base position to return to on shutdown is the open position.
        self.base_position = self.open_position

    def get_command(self, t):
        """시간(t)에 따라 각 관절의 목표 위치를 계산합니다."""
        # 각 관절의 위상차를 적용한 진동 값을 계산 (-1 ~ 1)
        oscillation = np.sin(self.frequency * t + self.phase_offsets)
        # 진동 값의 부호에 따라 열림/닫힘 상태를 결정
        command = np.where(oscillation > 0, self.close_position, self.open_position)
        return command * self.output_filter


class MotionGeneratorPlexus:
    """
    Generates joint commands for an opening and closing motion,
    specifically tailored for the Allegro Hand Plexus.
    """

    def __init__(self):
        # 각 관절의 움직임 범위 설정 (0도 ~ 30도)
        ten_degrees_in_rad = 20.0 * np.pi / 180.0
        self.open_position = np.array([0.0] * 16)
        self.close_position = np.array([ten_degrees_in_rad] * 16)

        # 움직임의 속도를 조절하는 주파수 (rad/s)
        # self.frequency = 2.0
        self.frequency = 4.0

        # 각 관절에 위상차를 적용하여 순차적으로 움직이게 합니다.
        # 16개 관절에 걸쳐 2*PI 만큼의 위상차를 균등하게 분배합니다.
        self.phase_offsets = np.linspace(0, 2 * np.pi, 16, endpoint=False)

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

    def get_command(self, t):
        """시간(t)에 따라 각 관절의 목표 위치를 계산합니다."""
        # 각 관절의 위상차를 적용한 진동 값을 계산 (-1 ~ 1)
        oscillation = np.sin(self.frequency * t + self.phase_offsets)

        # 진동 값의 부호에 따라 열림/닫힘 상태를 결정 (Step function)
        # oscillation > 0 이면 닫힘, 아니면 열림 상태로 설정
        command = np.where(oscillation > 0, self.close_position, self.open_position)

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

        # 부드러운 모션을 위한 파라미터
        self.update_rate = 50  # Hz
        self.timer_period = 1.0 / self.update_rate

        self.timer = self.create_timer(self.timer_period, self.motion_callback)
        self.get_logger().info(
            "Allegro Hand Motion Controller Node Started "
            f"(Update rate: {self.update_rate}Hz)"
        )

        # 연속적인 움직임을 위한 시간 변수
        self.t = 0.0

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
        타이머에 의해 주기적으로 호출되어 새로운 관절 명령을 계산하고 발행합니다.
        """
        pos_msg = Float64MultiArray()

        # 시간 변수를 증가시켜 파형을 진행시킵니다.
        self.t += self.timer_period

        # 현재 시간에 맞는 관절 위치 명령을 생성합니다.
        current_command = self.motion_generator.get_command(self.t)
        pos_msg.data = current_command.tolist()
        self.cmd_publisher_.publish(pos_msg)


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
