"""Start/unpause MoveIt Servo in workshop sim mode."""

from __future__ import annotations

from rclpy.node import Node
import rclpy
from std_srvs.srv import Trigger


class ServoTwistEnable(Node):
    def __init__(self) -> None:
        super().__init__("servo_twist_enable")
        self.declare_parameter("start_service", "/servo_node/start_servo")
        self.declare_parameter("unpause_service", "/servo_node/unpause_servo")
        self.declare_parameter("retry_period_sec", 0.5)

        start_service = self.get_parameter("start_service").get_parameter_value().string_value
        unpause_service = (
            self.get_parameter("unpause_service").get_parameter_value().string_value
        )
        retry_period = self.get_parameter("retry_period_sec").get_parameter_value().double_value

        self._servo_clients = [
            ("start", self.create_client(Trigger, start_service)),
            ("unpause", self.create_client(Trigger, unpause_service)),
        ]
        self._done: set[str] = set()
        self._request_in_flight = None
        self._request_name = ""
        self._timer = self.create_timer(max(retry_period, 0.1), self._tick)

    def _tick(self) -> None:
        if self._request_in_flight is not None:
            if not self._request_in_flight.done():
                return

            result = self._request_in_flight.result()
            if result is not None and result.success:
                self.get_logger().info(f"MoveIt Servo {self._request_name}: {result.message}")
                self._done.add(self._request_name)
            else:
                self.get_logger().warn(f"MoveIt Servo {self._request_name} failed, retrying")

            self._request_in_flight = None
            self._request_name = ""

        for name, client in self._servo_clients:
            if name in self._done:
                continue

            if not client.service_is_ready():
                self.get_logger().info(f"Waiting for Servo {name} service")
                return

            self._request_name = name
            self._request_in_flight = client.call_async(Trigger.Request())
            return

        self.get_logger().info("MoveIt Servo is started and unpaused")
        self._timer.cancel()


def main() -> None:
    rclpy.init()
    node = ServoTwistEnable()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
