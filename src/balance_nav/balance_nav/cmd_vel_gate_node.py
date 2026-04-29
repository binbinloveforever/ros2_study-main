import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, Twist
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu


def quaternion_to_pitch(quaternion: Quaternion) -> float:
    sinp = 2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
    if abs(sinp) >= 1.0:
        return math.copysign(math.pi / 2.0, sinp)
    return math.asin(sinp)


@dataclass
class TimedTwist:
    stamp: Time
    msg: Twist


class CmdVelGateNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_gate")

        self.declare_parameter("manual_cmd_topic", "/cmd_vel")
        self.declare_parameter("nav_cmd_topic", "/cmd_vel_nav")
        self.declare_parameter("safe_cmd_topic", "/cmd_vel_safe")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("manual_timeout", 0.35)
        self.declare_parameter("nav_timeout", 0.35)
        self.declare_parameter("max_forward_speed", 0.18)
        self.declare_parameter("max_reverse_speed", 0.08)
        self.declare_parameter("max_angular_speed", 0.6)
        self.declare_parameter("max_linear_accel", 0.5)
        self.declare_parameter("max_angular_accel", 1.0)
        self.declare_parameter("max_pitch_for_motion_deg", 20.0)

        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.manual_timeout = float(self.get_parameter("manual_timeout").value)
        self.nav_timeout = float(self.get_parameter("nav_timeout").value)
        self.max_forward_speed = float(self.get_parameter("max_forward_speed").value)
        self.max_reverse_speed = float(self.get_parameter("max_reverse_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.max_linear_accel = float(self.get_parameter("max_linear_accel").value)
        self.max_angular_accel = float(self.get_parameter("max_angular_accel").value)
        self.max_pitch_for_motion_deg = float(self.get_parameter("max_pitch_for_motion_deg").value)

        manual_cmd_topic = str(self.get_parameter("manual_cmd_topic").value)
        nav_cmd_topic = str(self.get_parameter("nav_cmd_topic").value)
        safe_cmd_topic = str(self.get_parameter("safe_cmd_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)

        self.manual_cmd: Optional[TimedTwist] = None
        self.nav_cmd: Optional[TimedTwist] = None
        self.pitch_deg = 0.0
        self.last_output_linear = 0.0
        self.last_output_angular = 0.0
        self.last_publish_time: Optional[Time] = None

        self.safe_cmd_publisher = self.create_publisher(Twist, safe_cmd_topic, 10)
        self.create_subscription(Twist, manual_cmd_topic, self.manual_cmd_callback, 20)
        self.create_subscription(Twist, nav_cmd_topic, self.nav_cmd_callback, 20)
        self.create_subscription(Imu, imu_topic, self.imu_callback, 20)
        self.create_timer(1.0 / max(self.publish_rate, 1.0), self.publish_safe_command)

        self.get_logger().info(
            f"cmd_vel_gate ready: manual={manual_cmd_topic}, nav={nav_cmd_topic}, out={safe_cmd_topic}"
        )

    def manual_cmd_callback(self, msg: Twist) -> None:
        self.manual_cmd = TimedTwist(self.get_clock().now(), msg)

    def nav_cmd_callback(self, msg: Twist) -> None:
        self.nav_cmd = TimedTwist(self.get_clock().now(), msg)

    def imu_callback(self, msg: Imu) -> None:
        self.pitch_deg = math.degrees(quaternion_to_pitch(msg.orientation))

    def command_is_fresh(self, timed_cmd: Optional[TimedTwist], timeout: float, now: Time) -> bool:
        if timed_cmd is None:
            return False
        return (now - timed_cmd.stamp).nanoseconds / 1e9 <= timeout

    def select_target_command(self, now: Time) -> Twist:
        if self.command_is_fresh(self.manual_cmd, self.manual_timeout, now):
            return self.manual_cmd.msg
        if self.command_is_fresh(self.nav_cmd, self.nav_timeout, now):
            return self.nav_cmd.msg
        return Twist()

    def clamp_target(self, target: Twist) -> Twist:
        safe_target = Twist()
        safe_target.linear.x = max(-self.max_reverse_speed, min(self.max_forward_speed, target.linear.x))
        safe_target.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, target.angular.z))

        if abs(self.pitch_deg) > self.max_pitch_for_motion_deg:
            safe_target.linear.x = 0.0
            safe_target.angular.z = 0.0

        return safe_target

    def slew_limit(self, desired: float, current: float, max_delta: float) -> float:
        return max(current - max_delta, min(current + max_delta, desired))

    def publish_safe_command(self) -> None:
        now = self.get_clock().now()
        if self.last_publish_time is None:
            self.last_publish_time = now
            return

        dt = (now - self.last_publish_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        target = self.clamp_target(self.select_target_command(now))

        limited = Twist()
        limited.linear.x = self.slew_limit(
            target.linear.x, self.last_output_linear, self.max_linear_accel * dt
        )
        limited.angular.z = self.slew_limit(
            target.angular.z, self.last_output_angular, self.max_angular_accel * dt
        )

        self.last_output_linear = limited.linear.x
        self.last_output_angular = limited.angular.z
        self.last_publish_time = now
        self.safe_cmd_publisher.publish(limited)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
