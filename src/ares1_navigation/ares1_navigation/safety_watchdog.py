#!/usr/bin/env python3
"""ARES-1 Safety Watchdog Node."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


class SafetyWatchdog(Node):
    def __init__(self):
        super().__init__('safety_watchdog')

        self.declare_parameter('sensor_timeout', 2.0)
        self.declare_parameter('max_vx', 2.0)
        self.declare_parameter('max_vy', 1.5)
        self.declare_parameter('max_wz', 2.0)
        self.declare_parameter('emergency_stop_distance', 0.3)
        self.declare_parameter('check_rate', 10.0)

        self.sensor_timeout = self.get_parameter('sensor_timeout').value
        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_wz = self.get_parameter('max_wz').value
        self.estop_dist = self.get_parameter('emergency_stop_distance').value
        check_rate = self.get_parameter('check_rate').value

        self.last_scan_time = self.get_clock().now()
        self.last_imu_time = self.get_clock().now()
        self.last_odom_time = self.get_clock().now()

        self.scan_received = False
        self.imu_received = False
        self.odom_received = False

        self.emergency = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(Bool, '/safety/status', 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)

        self.check_timer = self.create_timer(
            1.0 / check_rate, self.check_callback)

        self.get_logger().info('Safety Watchdog started')

    def scan_callback(self, msg: LaserScan):
        self.last_scan_time = self.get_clock().now()
        self.scan_received = True

        if len(msg.ranges) > 0:
            front_ranges = []
            for i, r in enumerate(msg.ranges):
                angle = msg.angle_min + i * msg.angle_increment
                if -0.5 < angle < 0.5 and msg.range_min < r < msg.range_max:
                    front_ranges.append(r)

            if front_ranges:
                min_front = min(front_ranges)
                if min_front < self.estop_dist:
                    if not self.emergency:
                        self.get_logger().error(
                            f'EMERGENCY STOP: obstacle at {min_front:.2f}m')
                    self.emergency = True
                else:
                    self.emergency = False

    def imu_callback(self, msg: Imu):
        self.last_imu_time = self.get_clock().now()
        self.imu_received = True

    def odom_callback(self, msg: Odometry):
        self.last_odom_time = self.get_clock().now()
        self.odom_received = True

        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z

        if (abs(vx) > self.max_vx or
            abs(vy) > self.max_vy or
            abs(wz) > self.max_wz):
            self.get_logger().error(
                f'Velocity limit exceeded: vx={vx:.2f}, vy={vy:.2f}, '
                f'wz={wz:.2f}')
            self.emergency = True

    def check_callback(self):
        now = self.get_clock().now()

        scan_elapsed = (now - self.last_scan_time).nanoseconds / 1e9
        imu_elapsed = (now - self.last_imu_time).nanoseconds / 1e9
        odom_elapsed = (now - self.last_odom_time).nanoseconds / 1e9

        if self.scan_received and scan_elapsed > self.sensor_timeout:
            self.get_logger().error(f'LiDAR timeout ({scan_elapsed:.1f}s)')
            self.emergency = True

        if self.imu_received and imu_elapsed > self.sensor_timeout:
            self.get_logger().error(f'IMU timeout ({imu_elapsed:.1f}s)')
            self.emergency = True

        if self.odom_received and odom_elapsed > self.sensor_timeout:
            self.get_logger().error(f'Odometry timeout ({odom_elapsed:.1f}s)')
            self.emergency = True

        try:
            self.tf_buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time())
        except Exception:
            pass

        status = Bool()
        status.data = not self.emergency
        self.status_pub.publish(status)

        if self.emergency:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
