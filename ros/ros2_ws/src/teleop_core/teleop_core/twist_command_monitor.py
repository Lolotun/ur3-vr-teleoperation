"""Monitor and clamp VR Cartesian twist commands."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TwistStamped, Vector3
from rclpy.node import Node
from std_msgs.msg import String


def _norm(v: Vector3) -> float:
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def _clamp_vector(v: Vector3, max_norm: float) -> tuple[Vector3, bool]:
    norm = _norm(v)
    if norm <= max_norm or norm < 1e-9:
        return v, False

    scale = max_norm / norm
    return Vector3(x=v.x * scale, y=v.y * scale, z=v.z * scale), True


class TwistCommandMonitor(Node):
    def __init__(self) -> None:
        super().__init__("twist_command_monitor")

        self.declare_parameter("input_topic", "/team_1/twist_command")
        self.declare_parameter("output_topic", "/team_1/twist_command_safe")
        self.declare_parameter("safety_status_topic", "/team_1/safety_status")
        self.declare_parameter("max_linear_speed", 0.12)
        self.declare_parameter("max_angular_speed", 0.0)
        self.declare_parameter("log_every_n", 30)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        status_topic = self.get_parameter("safety_status_topic").get_parameter_value().string_value

        self._max_linear_speed = (
            self.get_parameter("max_linear_speed").get_parameter_value().double_value
        )
        self._max_angular_speed = (
            self.get_parameter("max_angular_speed").get_parameter_value().double_value
        )
        self._log_every_n = (
            self.get_parameter("log_every_n").get_parameter_value().integer_value
        )
        self._count = 0

        self._pub = self.create_publisher(TwistStamped, output_topic, 10)
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(TwistStamped, input_topic, self._on_twist, 10)

        self.get_logger().info(
            f"Twist monitor: {input_topic} -> {output_topic}, "
            f"linear <= {self._max_linear_speed:.3f} m/s"
        )

    def _on_twist(self, msg: TwistStamped) -> None:
        safe = TwistStamped()
        safe.header = msg.header
        safe.header.stamp = self.get_clock().now().to_msg()
        safe.twist.linear, linear_clamped = _clamp_vector(
            msg.twist.linear,
            self._max_linear_speed,
        )
        safe.twist.angular, angular_clamped = _clamp_vector(
            msg.twist.angular,
            self._max_angular_speed,
        )
        self._pub.publish(safe)

        status = "TWIST_CLAMP" if linear_clamped or angular_clamped else "TWIST_OK"
        self._status_pub.publish(String(data=status))

        self._count += 1
        if self._log_every_n > 0 and self._count % self._log_every_n == 0:
            lin = safe.twist.linear
            ang = safe.twist.angular
            self.get_logger().info(
                f"{status}: frame={safe.header.frame_id} "
                f"linear=({lin.x:.3f}, {lin.y:.3f}, {lin.z:.3f}) "
                f"angular=({ang.x:.3f}, {ang.y:.3f}, {ang.z:.3f})"
            )


def main() -> None:
    rclpy.init()
    node = TwistCommandMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
