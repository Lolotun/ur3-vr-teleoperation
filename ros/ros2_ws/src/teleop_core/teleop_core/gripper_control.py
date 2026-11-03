"""Gripper command parsing and latest-position coalescing, without ROS or sockets."""
import math


def clamp_byte(value):
    return max(0, min(255, int(value)))


def parse_position(command, open_position=0, close_position=255):
    command = command.strip().lower()
    if command in ('open', 'release', '0'):
        return open_position
    if command in ('close', 'grab', 'grip', '1'):
        return close_position
    try:
        value = float(command)
    except ValueError:
        return None
    if not math.isfinite(value):
        return None
    if 0. <= value <= 1.:
        return clamp_byte(round(open_position + value * (close_position - open_position)))
    return clamp_byte(round(value))


class LatestGripperCommand:
    """Keep only the latest requested position; acknowledge successful sends."""
    def __init__(self):
        self.pending = None
        self.sent = None

    def offer(self, position):
        self.pending = position

    def take(self):
        position, self.pending = self.pending, None
        return position if position != self.sent else None

    def acknowledge(self, position):
        self.sent = position
