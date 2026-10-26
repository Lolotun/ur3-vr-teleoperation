"""Publish the current MoveIt/TF TCP pose for the Unity debug marker."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class TcpActualPublisher(Node):
    def __init__(self) -> None:
        super().__init__("tcp_actual_publisher")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tcp_frame", "tool0")
        self.declare_parameter("output_topic", "/team_1/tcp_actual")
        self.declare_parameter("publish_hz", 30.0)

        self._base_frame = self.get_parameter("base_frame").value
        self._tcp_frame = self.get_parameter("tcp_frame").value
        output_topic = self.get_parameter("output_topic").value
        publish_hz = self.get_parameter("publish_hz").value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(PoseStamped, output_topic, 10)
        self.create_timer(1.0 / max(float(publish_hz), 1.0), self._publish_pose)

        self.get_logger().info(
            f"Actual TCP: {self._base_frame} -> {self._tcp_frame} -> {output_topic}"
        )

    def _publish_pose(self) -> None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._tcp_frame,
                Time(),
            )
        except Exception:
            # TF is unavailable briefly while robot_state_publisher starts.
            return

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._base_frame
        message.pose.position.x = transform.transform.translation.x
        message.pose.position.y = transform.transform.translation.y
        message.pose.position.z = transform.transform.translation.z
        message.pose.orientation = transform.transform.rotation
        self._publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = TcpActualPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
