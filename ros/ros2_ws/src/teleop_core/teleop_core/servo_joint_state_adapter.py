"""Tiny simulated controller for MoveIt Servo workshop mode."""

from __future__ import annotations

from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

from teleop_core.ur3_model import DEFAULT_SEED, UR3_ARM_JOINTS


class ServoJointStateAdapter(Node):
    def __init__(self) -> None:
        super().__init__("servo_joint_state_adapter")

        self.declare_parameter("trajectory_topic", "/team_1/servo_joint_trajectory")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("initial_positions", list(DEFAULT_SEED))

        trajectory_topic = (
            self.get_parameter("trajectory_topic").get_parameter_value().string_value
        )
        joint_states_topic = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )
        hz = self.get_parameter("publish_hz").get_parameter_value().double_value
        initial_positions = list(
            self.get_parameter("initial_positions")
            .get_parameter_value()
            .double_array_value
        )
        if len(initial_positions) != len(UR3_ARM_JOINTS):
            self.get_logger().warn(
                "initial_positions must contain 6 values; falling back to DEFAULT_SEED"
            )
            initial_positions = list(DEFAULT_SEED)

        self._joint_names: List[str] = list(UR3_ARM_JOINTS)
        self._positions: List[float] = initial_positions
        self._velocities: List[float] = [0.0] * len(self._joint_names)

        self._joint_pub = self.create_publisher(JointState, joint_states_topic, 10)
        self.create_subscription(JointTrajectory, trajectory_topic, self._on_trajectory, 10)
        self.create_timer(1.0 / max(hz, 1.0), self._publish_joint_state)

        self.get_logger().info(
            f"Servo sim adapter: {trajectory_topic} -> {joint_states_topic} @ {hz:.1f} Hz"
        )

    def _on_trajectory(self, msg: JointTrajectory) -> None:
        if not msg.joint_names or not msg.points:
            return

        point = msg.points[-1]
        if not point.positions:
            return

        name_to_index = {name: i for i, name in enumerate(msg.joint_names)}
        for idx, joint_name in enumerate(self._joint_names):
            source_idx = name_to_index.get(joint_name)
            if source_idx is None or source_idx >= len(point.positions):
                continue

            self._positions[idx] = float(point.positions[source_idx])
            if source_idx < len(point.velocities):
                self._velocities[idx] = float(point.velocities[source_idx])
            else:
                self._velocities[idx] = 0.0

    def _publish_joint_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_names)
        msg.position = list(self._positions)
        msg.velocity = list(self._velocities)
        msg.effort = []
        self._joint_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ServoJointStateAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
