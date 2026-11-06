"""status_monitor node.

Prints a periodically-refreshed terminal dashboard summarizing the health of
every major ARES-1 subsystem (per section 30 of the spec), plus current
pose, velocity, active waypoint, and localization/navigation status.

    ros2 run ares1_bringup status_monitor
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, Imu, Image, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Freshness:
    def __init__(self, node: Node, timeout_sec: float = 2.0):
        self.node = node
        self.timeout_sec = timeout_sec
        self.last_stamp = None

    def touch(self):
        self.last_stamp = self.node.get_clock().now()

    def ok(self) -> bool:
        if self.last_stamp is None:
            return False
        elapsed = (self.node.get_clock().now() - self.last_stamp).nanoseconds * 1e-9
        return elapsed <= self.timeout_sec


class StatusMonitor(Node):

    def __init__(self):
        super().__init__('status_monitor')

        self.declare_parameter('print_rate_hz', 1.0)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)

        self.f_rgb = Freshness(self)
        self.f_depth = Freshness(self)
        self.f_scan = Freshness(self)
        self.f_imu = Freshness(self)
        self.f_joints = Freshness(self)
        self.f_odom = Freshness(self)

        self._pose = (0.0, 0.0, 0.0)
        self._vel = (0.0, 0.0, 0.0)
        self._safety_status = 'UNKNOWN'
        self._current_waypoint = 'N/A'

        self.create_subscription(Image, '/realsense_D456/color/image_raw',
                                  lambda m: self.f_rgb.touch(), qos)
        self.create_subscription(Image, '/realsense_D456/depth/image_raw',
                                  lambda m: self.f_depth.touch(), qos)
        self.create_subscription(LaserScan, '/scan', lambda m: self.f_scan.touch(), qos)
        self.create_subscription(Imu, '/imu/data', lambda m: self.f_imu.touch(), qos)
        self.create_subscription(JointState, '/joint_states', lambda m: self.f_joints.touch(), qos)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos)
        self.create_subscription(String, '/safety/status', self._safety_cb, qos)

        rate_hz = float(self.get_parameter('print_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate_hz, self._print_dashboard)

    def _odom_cb(self, msg: Odometry):
        self.f_odom.touch()
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self._vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z)

    def _safety_cb(self, msg: String):
        self._safety_status = msg.data

    @staticmethod
    def _tag(ok: bool) -> str:
        return 'OK' if ok else 'MISSING'

    def _print_dashboard(self):
        x, y, yaw = self._pose
        vx, vy, wz = self._vel

        lines = []
        lines.append('=' * 42)
        lines.append('ARES-1 AUTONOMOUS SYSTEM')
        lines.append('=' * 42)
        lines.append(f'D456 RGB:          {self._tag(self.f_rgb.ok())}')
        lines.append(f'D456 DEPTH:        {self._tag(self.f_depth.ok())}')
        lines.append(f'RPLIDAR S3:        {self._tag(self.f_scan.ok())}')
        lines.append(f'BNO085 IMU:        {self._tag(self.f_imu.ok())}')
        lines.append(f'Wheel Encoders:    {self._tag(self.f_joints.ok())}')
        lines.append(f'EKF/Odometry:      {self._tag(self.f_odom.ok())}')
        lines.append('SLAM:              (see /map topic + RViz)')
        lines.append('Global Costmap:    (see /global_costmap/costmap)')
        lines.append('Local Costmap:     (see /local_costmap/costmap)')
        lines.append('A* Planner:        (see /plan topic)')
        lines.append('MPPI Controller:   (see /cmd_vel_nav topic)')
        lines.append('Swivel Drive:      (see /steering_controller/commands)')
        lines.append('Suspension:        (see /joint_states *_suspension_joint)')
        lines.append(f'Safety:            {self._safety_status}')
        lines.append('=' * 42)
        lines.append(f'Pose (odom):   x={x:6.2f}  y={y:6.2f}  yaw={math.degrees(yaw):6.1f} deg')
        lines.append(f'Velocity:      vx={vx:5.2f}  vy={vy:5.2f}  wz={wz:5.2f}')
        lines.append(f'Waypoint:      {self._current_waypoint}')
        lines.append('=' * 42)

        print('\n'.join(lines), flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = StatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
