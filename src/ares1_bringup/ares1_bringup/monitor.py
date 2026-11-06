#!/usr/bin/env python3
"""ARES-1 System Monitor."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
import math


class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')

        self.declare_parameter('print_rate', 1.0)
        print_rate = self.get_parameter('print_rate').value

        self.scan_ok = False
        self.imu_ok = False
        self.odom_ok = False
        self.joint_ok = False
        self.cmd_vel_received = False
        self.tf_ok = False

        self.current_pose = None
        self.current_vel = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self._imu_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.timer = self.create_timer(1.0 / print_rate, self.print_status)

        self.get_logger().info('System Monitor started')

    def _scan_cb(self, msg):
        self.scan_ok = True

    def _imu_cb(self, msg):
        self.imu_ok = True

    def _joint_cb(self, msg):
        self.joint_ok = True

    def odom_callback(self, msg: Odometry):
        self.odom_ok = True
        self.current_pose = msg.pose.pose
        self.current_vel = msg.twist.twist

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vel_received = True

    def print_status(self):
        try:
            self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
            self.tf_ok = True
        except Exception:
            self.tf_ok = False

        lines = []
        lines.append('=' * 48)
        lines.append('  ARES-1 AUTONOMOUS SYSTEM')
        lines.append('=' * 48)
        lines.append(f'  D456 RGB:          {"OK" if self.scan_ok else "WAIT"}')
        lines.append(f'  D456 DEPTH:        {"OK" if self.odom_ok else "WAIT"}')
        lines.append(f'  RPLIDAR S3:        {"OK" if self.scan_ok else "WAIT"}')
        lines.append(f'  BNO085 IMU:        {"OK" if self.imu_ok else "WAIT"}')
        lines.append(f'  Wheel Encoders:    {"OK" if self.joint_ok else "WAIT"}')
        lines.append(f'  EKF:               {"OK" if self.odom_ok else "WAIT"}')
        lines.append(f'  TF odom->base:     {"OK" if self.tf_ok else "FAIL"}')
        lines.append(f'  cmd_vel:           {"OK" if self.cmd_vel_received else "WAIT"}')

        if self.current_pose:
            p = self.current_pose.position
            q = self.current_pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            lines.append(f'  Pose:  x={p.x:.2f} y={p.y:.2f} '
                         f'yaw={math.degrees(yaw):.1f}deg')

        if self.current_vel:
            v = self.current_vel
            lines.append(f'  Vel:   vx={v.linear.x:.2f} vy={v.linear.y:.2f} '
                         f'wz={v.angular.z:.2f}')

        lines.append('=' * 48)

        print('\033[2J\033[H', end='')
        print('\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
