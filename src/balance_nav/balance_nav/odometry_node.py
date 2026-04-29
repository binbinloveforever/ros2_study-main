import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster


def quaternion_to_yaw(quaternion: Quaternion) -> float:
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half_yaw = yaw * 0.5
    msg = Quaternion()
    msg.w = math.cos(half_yaw)
    msg.z = math.sin(half_yaw)
    return msg


class BalanceOdometryNode(Node):
    def __init__(self) -> None:
        super().__init__("balance_odometry")

        self.declare_parameter("wheel_radius", 0.1)
        self.declare_parameter("wheel_separation", 0.37)
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint_link")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("left_wheel_joint", "left_wheel_joint")
        self.declare_parameter("right_wheel_joint", "right_wheel_joint")
        self.declare_parameter("publish_tf", True)

        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.left_wheel_joint = str(self.get_parameter("left_wheel_joint").value)
        self.right_wheel_joint = str(self.get_parameter("right_wheel_joint").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)

        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0
        self.yaw = 0.0
        self.yaw_rate = 0.0
        self.x = 0.0
        self.y = 0.0

        self.have_joint_state = False
        self.have_imu = False
        self.last_update_time: Optional[Time] = None

        self.odom_publisher = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.create_subscription(JointState, joint_states_topic, self.joint_states_callback, 20)
        self.create_subscription(Imu, imu_topic, self.imu_callback, 20)
        self.create_timer(1.0 / max(self.publish_rate, 1.0), self.publish_odometry)

        self.get_logger().info(
            f"balance_odometry ready: joints=({self.left_wheel_joint}, {self.right_wheel_joint}), "
            f"wheel_radius={self.wheel_radius:.3f}, wheel_separation={self.wheel_separation:.3f}"
        )

    def joint_states_callback(self, msg: JointState) -> None:
        velocity_by_name = {
            name: msg.velocity[index]
            for index, name in enumerate(msg.name)
            if index < len(msg.velocity)
        }

        if self.left_wheel_joint in velocity_by_name:
            self.left_wheel_velocity = velocity_by_name[self.left_wheel_joint]
        if self.right_wheel_joint in velocity_by_name:
            self.right_wheel_velocity = velocity_by_name[self.right_wheel_joint]

        self.have_joint_state = (
            self.left_wheel_joint in velocity_by_name and self.right_wheel_joint in velocity_by_name
        )

    def imu_callback(self, msg: Imu) -> None:
        self.yaw = quaternion_to_yaw(msg.orientation)
        self.yaw_rate = msg.angular_velocity.z
        self.have_imu = True

    def publish_odometry(self) -> None:
        if not self.have_joint_state or not self.have_imu:
            return

        now = self.get_clock().now()
        if self.last_update_time is None:
            self.last_update_time = now
            return

        dt = (now - self.last_update_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        linear_velocity = self.wheel_radius * (
            self.left_wheel_velocity + self.right_wheel_velocity
        ) * 0.5
        angular_velocity = self.yaw_rate

        self.x += linear_velocity * math.cos(self.yaw) * dt
        self.y += linear_velocity * math.sin(self.yaw) * dt
        self.last_update_time = now

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = yaw_to_quaternion(self.yaw)
        odom_msg.twist.twist.linear.x = linear_velocity
        odom_msg.twist.twist.angular.z = angular_velocity

        odom_msg.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2,
        ]
        odom_msg.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.3,
        ]

        self.odom_publisher.publish(odom_msg)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = now.to_msg()
            transform.header.frame_id = self.odom_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation = odom_msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BalanceOdometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
