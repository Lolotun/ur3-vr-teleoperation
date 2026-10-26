"""Best-effort Wi-Fi fallback that sends slow speedl commands to URScript."""

from __future__ import annotations

import math
import socket
import threading
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistStamped, Vector3
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


def _norm(v: Vector3) -> float:
    return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)


def _scale_to_limit(v: Vector3, max_norm: float) -> tuple[float, float, float]:
    norm = _norm(v)
    if norm <= max_norm or norm < 1e-9:
        return v.x, v.y, v.z
    scale = max_norm / norm
    return v.x * scale, v.y * scale, v.z * scale


class UrSpeedlWifiBridge(Node):
    def __init__(self) -> None:
        super().__init__("ur_speedl_wifi_bridge")
        dynamic_param = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter("robot_ip", "")
        self.declare_parameter("robot_port", 30002)
        self.declare_parameter("input_topic", "/team_1/twist_command_safe")
        self.declare_parameter("send_hz", 6.0, dynamic_param)
        self.declare_parameter("max_linear_speed", 0.010, dynamic_param)
        self.declare_parameter("max_angular_speed", 0.0, dynamic_param)
        self.declare_parameter("acceleration", 0.05, dynamic_param)
        self.declare_parameter("command_duration", 0.06, dynamic_param)
        self.declare_parameter("stale_timeout_sec", 0.12, dynamic_param)
        self.declare_parameter("connect_timeout_sec", 0.04, dynamic_param)
        self.declare_parameter("deadband", 0.002, dynamic_param)
        self.declare_parameter("linear_x_sign", -1.0, dynamic_param)
        self.declare_parameter("linear_y_sign", -1.0, dynamic_param)
        self.declare_parameter("linear_z_sign", 1.0, dynamic_param)
        self.declare_parameter("angular_x_sign", 1.0, dynamic_param)
        self.declare_parameter("angular_y_sign", 1.0, dynamic_param)
        self.declare_parameter("angular_z_sign", 1.0, dynamic_param)
        self.declare_parameter("log_every_n", 10)

        self._robot_ip = self.get_parameter("robot_ip").value
        self._robot_port = int(self.get_parameter("robot_port").value)
        input_topic = self.get_parameter("input_topic").value
        send_hz = float(self.get_parameter("send_hz").value)
        self._max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self._max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self._acceleration = float(self.get_parameter("acceleration").value)
        self._command_duration = float(self.get_parameter("command_duration").value)
        self._stale_timeout_sec = float(self.get_parameter("stale_timeout_sec").value)
        self._connect_timeout_sec = float(self.get_parameter("connect_timeout_sec").value)
        self._deadband = float(self.get_parameter("deadband").value)
        self._linear_signs = (
            float(self.get_parameter("linear_x_sign").value),
            float(self.get_parameter("linear_y_sign").value),
            float(self.get_parameter("linear_z_sign").value),
        )
        self._angular_signs = (
            float(self.get_parameter("angular_x_sign").value),
            float(self.get_parameter("angular_y_sign").value),
            float(self.get_parameter("angular_z_sign").value),
        )
        self._log_every_n = int(self.get_parameter("log_every_n").value)

        if not self._robot_ip:
            raise RuntimeError("robot_ip parameter is required")

        self._latest_msg: Optional[TwistStamped] = None
        self._latest_time = self.get_clock().now()
        self._send_count = 0
        self._fail_count = 0
        self._last_nonzero = False
        self._sock: Optional[socket.socket] = None
        self._sock_lock = threading.Lock()

        self.create_subscription(TwistStamped, input_topic, self._on_twist, 10)
        self.create_timer(1.0 / max(send_hz, 1.0), self._on_timer)

        self.get_logger().warn(
            "Wi-Fi speedl fallback active. This bypasses External Control and "
            "sends slow URScript speedl commands to port "
            f"{self._robot_port}. Keep speed slider low and hand on E-stop."
        )
        self.get_logger().info(
            f"{input_topic} -> speedl on {self._robot_ip}:{self._robot_port}, "
            f"linear <= {self._max_linear_speed:.3f} m/s, "
            f"send_hz={send_hz:.1f}, t={self._command_duration:.2f}s"
        )

    def _on_twist(self, msg: TwistStamped) -> None:
        self._latest_msg = msg
        self._latest_time = self.get_clock().now()

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        age = (now - self._latest_time).nanoseconds / 1e9

        if self._latest_msg is None or age > self._stale_timeout_sec:
            linear = (0.0, 0.0, 0.0)
            angular = (0.0, 0.0, 0.0)
        else:
            linear = _scale_to_limit(self._latest_msg.twist.linear, self._max_linear_speed)
            angular = _scale_to_limit(self._latest_msg.twist.angular, self._max_angular_speed)
            if math.sqrt(sum(value * value for value in linear)) < self._deadband:
                linear = (0.0, 0.0, 0.0)
            if math.sqrt(sum(value * value for value in angular)) < self._deadband:
                angular = (0.0, 0.0, 0.0)

        linear = tuple(value * sign for value, sign in zip(linear, self._linear_signs))
        angular = tuple(value * sign for value, sign in zip(angular, self._angular_signs))

        is_nonzero = any(abs(value) > 1e-9 for value in (*linear, *angular))
        if is_nonzero:
            self._send_speedl(linear, angular)
            self._last_nonzero = True
        elif self._last_nonzero:
            self._send_stop()
            self._last_nonzero = False

    def _send_speedl(
        self,
        linear: tuple[float, float, float],
        angular: tuple[float, float, float],
    ) -> None:
        vx, vy, vz = linear
        rx, ry, rz = angular
        script = (
            "def wifi_speedl_cmd():\n"
            f"  speedl([{vx:.6f}, {vy:.6f}, {vz:.6f}, "
            f"{rx:.6f}, {ry:.6f}, {rz:.6f}], "
            f"a={self._acceleration:.6f}, t={self._command_duration:.6f})\n"
            "end\n"
        )
        log_text = (
            "speedl sent: "
            f"linear=({vx:.3f}, {vy:.3f}, {vz:.3f}), "
            f"angular=({rx:.3f}, {ry:.3f}, {rz:.3f})"
        )
        self._send_script(script, "speedl", log_text)

    def _send_stop(self) -> None:
        script = (
            "def wifi_speedl_stop():\n"
            "  speedl([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], a=0.2, t=0.02)\n"
            "  stopl(0.5)\n"
            "end\n"
        )
        self._send_script(script, "stop", "stop sent")

    def _send_script(self, script: str, label: str, log_text: str) -> None:
        try:
            sock = self._get_socket()
            sock.sendall(script.encode("ascii"))
        except OSError as exc:
            self._close_socket()
            self._fail_count += 1
            if self._fail_count == 1 or self._fail_count % self._log_every_n == 0:
                self.get_logger().warn(f"{label} send failed: {exc}")
            return

        self._send_count += 1
        self._fail_count = 0
        if label == "speedl" and self._log_every_n > 0 and self._send_count % self._log_every_n == 0:
            self.get_logger().info(log_text)
        elif label == "stop":
            self.get_logger().info(log_text)

    def _get_socket(self) -> socket.socket:
        with self._sock_lock:
            if self._sock is not None:
                return self._sock

            sock = socket.create_connection(
                (self._robot_ip, self._robot_port),
                timeout=self._connect_timeout_sec,
            )
            sock.settimeout(self._connect_timeout_sec)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._sock = sock
            self.get_logger().info(
                f"Connected persistent URScript socket to {self._robot_ip}:{self._robot_port}"
            )
            return sock

    def _close_socket(self) -> None:
        with self._sock_lock:
            if self._sock is None:
                return
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    def destroy_node(self) -> bool:
        self._send_stop()
        self._close_socket()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = UrSpeedlWifiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
