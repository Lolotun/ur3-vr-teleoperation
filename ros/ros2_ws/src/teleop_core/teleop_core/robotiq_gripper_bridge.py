"""Small Robotiq 2F socket bridge for the Wi-Fi speedl demo path."""

from __future__ import annotations

import socket
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import String


class RobotiqGripperBridge(Node):
    def __init__(self) -> None:
        super().__init__("robotiq_gripper_bridge")
        dynamic_param = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter("robot_ip", "")
        self.declare_parameter("robot_port", 63352)
        self.declare_parameter("input_topic", "/team_1/gripper_command")
        self.declare_parameter("open_position", 0, dynamic_param)
        self.declare_parameter("close_position", 255, dynamic_param)
        self.declare_parameter("speed", 180, dynamic_param)
        self.declare_parameter("force", 20, dynamic_param)
        self.declare_parameter("connect_timeout_sec", 0.20, dynamic_param)
        self.declare_parameter("min_command_interval_sec", 0.20, dynamic_param)

        self._robot_ip = self.get_parameter("robot_ip").value
        self._robot_port = int(self.get_parameter("robot_port").value)
        input_topic = self.get_parameter("input_topic").value
        self._open_position = self._clamp_byte(self.get_parameter("open_position").value)
        self._close_position = self._clamp_byte(self.get_parameter("close_position").value)
        self._speed = self._clamp_byte(self.get_parameter("speed").value)
        self._force = self._clamp_byte(self.get_parameter("force").value)
        self._connect_timeout_sec = float(self.get_parameter("connect_timeout_sec").value)
        self._min_command_interval_sec = float(
            self.get_parameter("min_command_interval_sec").value
        )

        if not self._robot_ip:
            raise RuntimeError("robot_ip parameter is required")

        self._last_command = ""
        self._last_command_time = 0.0
        self._activated = False

        self.create_subscription(String, input_topic, self._on_command, 10)
        self.get_logger().info(
            f"{input_topic} -> Robotiq socket on {self._robot_ip}:{self._robot_port}, "
            f"open={self._open_position}, close={self._close_position}, "
            f"speed={self._speed}, force={self._force}"
        )

    def _on_command(self, msg: String) -> None:
        command = msg.data.strip().lower()
        now = time.monotonic()
        if (
            command == self._last_command
            and now - self._last_command_time < self._min_command_interval_sec
        ):
            return

        position = self._parse_position(command)
        if position is None:
            self.get_logger().warn(
                f"Ignoring gripper command '{msg.data}'. Use open, close, or 0..255."
            )
            return

        self._last_command = command
        self._last_command_time = now
        self._send_position(position)

    def _parse_position(self, command: str) -> int | None:
        if command in ("open", "release", "0"):
            return self._open_position
        if command in ("close", "grab", "grip", "1"):
            return self._close_position
        try:
            value = float(command)
        except ValueError:
            return None

        if 0.0 <= value <= 1.0:
            return self._clamp_byte(round(value * 255.0))
        return self._clamp_byte(round(value))

    def _send_position(self, position: int) -> None:
        lines = []
        if not self._activated:
            lines.extend(["SET ACT 1", "SET GTO 1"])
        lines.extend(
            [
                f"SET SPE {self._speed}",
                f"SET FOR {self._force}",
                f"SET POS {position}",
            ]
        )

        try:
            with socket.create_connection(
                (self._robot_ip, self._robot_port),
                timeout=self._connect_timeout_sec,
            ) as sock:
                sock.settimeout(self._connect_timeout_sec)
                for line in lines:
                    sock.sendall((line + "\n").encode("ascii"))
                    time.sleep(0.01)
        except OSError as exc:
            self.get_logger().warn(f"Robotiq command failed: {exc}")
            return

        self._activated = True
        self.get_logger().info(f"Robotiq position sent: {position}")

    @staticmethod
    def _clamp_byte(value: object) -> int:
        return max(0, min(255, int(value)))


def main() -> None:
    rclpy.init()
    node = RobotiqGripperBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
