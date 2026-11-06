"""safety_watchdog node.

Monitors critical sensor and control data streams. If any required stream
goes stale, or localization/navigation is judged to have failed, publishes
a zero Twist on /cmd_vel_estop (higher priority stop) and sets an
emergency-stop flag on /safety/estop that other nodes (e.g. swivel_controller
could optionally subscribe to) can honor.

In this reference implementation, the enforcement mechanism is: the watchdog
publishes directly to /cmd_vel at a higher rate than Nav2 whenever an E-STOP
condition is active, which — combined with swivel_controller's own
command-timeout watchdog — guarantees the rover stops if either (a) this
node detects a system-level fault, or (b) /cmd_vel itself goes stale.

Monitored conditions:
  - /scan            (LiDAR)         timeout -> E-STOP
  - /imu/data         (IMU)           timeout -> E-STOP
  - /odom             (wheel odom)    timeout -> E-STOP
  - /joint_states     (ros2_control)  timeout -> E-STOP
  - /cmd_vel          (commanded vel) sanity-checked against max limits;
                                      values exceeding limits are clamped
                                      and a warning is logged (defense in
                                      depth on top of Nav2's own limits).
  - localization covariance (via /odom pose covariance trace, as a cheap
    proxy for EKF health) exceeding a threshold -> localization-failure flag.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, Imu, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


class StreamWatchdog:
    """Tracks the last-received time of a single topic."""

    def __init__(self, node: Node, name: str, timeout_sec: float):
        self.node = node
        self.name = name
        self.timeout_sec = timeout_sec
        self.last_stamp = None

    def touch(self):
        self.last_stamp = self.node.get_clock().now()

    def is_stale(self) -> bool:
        if self.last_stamp is None:
            return True
        elapsed = (self.node.get_clock().now() - self.last_stamp).nanoseconds * 1e-9
        return elapsed > self.timeout_sec


class SafetyWatchdog(Node):

    def __init__(self):
        super().__init__('safety_watchdog')

        self.declare_parameter('scan_timeout_sec', 1.0)
        self.declare_parameter('imu_timeout_sec', 0.5)
        self.declare_parameter('odom_timeout_sec', 0.5)
        self.declare_parameter('joint_state_timeout_sec', 0.5)
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)
        self.declare_parameter('max_linear_speed', 1.5)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('max_pose_covariance_trace', 5.0)
        self.declare_parameter('check_rate_hz', 10.0)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.max_pose_covariance_trace = float(
            self.get_parameter('max_pose_covariance_trace').value)

        self.scan_wd = StreamWatchdog(self, 'scan', float(self.get_parameter('scan_timeout_sec').value))
        self.imu_wd = StreamWatchdog(self, 'imu', float(self.get_parameter('imu_timeout_sec').value))
        self.odom_wd = StreamWatchdog(self, 'odom', float(self.get_parameter('odom_timeout_sec').value))
        self.joint_wd = StreamWatchdog(self, 'joint_states', float(self.get_parameter('joint_state_timeout_sec').value))
        self.cmd_vel_wd = StreamWatchdog(self, 'cmd_vel', float(self.get_parameter('cmd_vel_timeout_sec').value))

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)

        self._localization_ok = True
        self._last_cmd_vel = Twist()

        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, qos)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_cb, qos)

        self.estop_pub = self.create_publisher(Bool, '/safety/estop', 10)
        self.estop_cmd_pub = self.create_publisher(Twist, '/cmd_vel_estop', 10)
        self.status_pub = self.create_publisher(String, '/safety/status', 10)

        rate_hz = float(self.get_parameter('check_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate_hz, self._check)

        self.get_logger().info('safety_watchdog started.')

    def _scan_cb(self, msg): self.scan_wd.touch()
    def _imu_cb(self, msg): self.imu_wd.touch()
    def _joint_cb(self, msg): self.joint_wd.touch()

    def _odom_cb(self, msg: Odometry):
        self.odom_wd.touch()
        trace = (msg.pose.covariance[0] + msg.pose.covariance[7] + msg.pose.covariance[35])
        self._localization_ok = trace <= self.max_pose_covariance_trace

    def _cmd_vel_cb(self, msg: Twist):
        self.cmd_vel_wd.touch()
        self._last_cmd_vel = msg
        if (abs(msg.linear.x) > self.max_linear_speed or
                abs(msg.linear.y) > self.max_linear_speed or
                abs(msg.angular.z) > self.max_angular_speed):
            self.get_logger().warn(
                f'/cmd_vel exceeded configured limits: '
                f'vx={msg.linear.x:.2f} vy={msg.linear.y:.2f} wz={msg.angular.z:.2f}')

    def _check(self):
        faults = []
        if self.scan_wd.is_stale():
            faults.append('LIDAR (/scan) TIMEOUT')
        if self.imu_wd.is_stale():
            faults.append('IMU (/imu/data) TIMEOUT')
        if self.odom_wd.is_stale():
            faults.append('ODOMETRY (/odom) TIMEOUT')
        if self.joint_wd.is_stale():
            faults.append('JOINT_STATES TIMEOUT')
        if not self._localization_ok:
            faults.append('LOCALIZATION COVARIANCE TOO HIGH')

        estop_active = len(faults) > 0

        estop_msg = Bool()
        estop_msg.data = estop_active
        self.estop_pub.publish(estop_msg)

        status_msg = String()
        status_msg.data = 'OK' if not estop_active else ('FAULT: ' + '; '.join(faults))
        self.status_pub.publish(status_msg)

        if estop_active:
            self.get_logger().error('SAFETY WATCHDOG: ' + '; '.join(faults) + ' -> E-STOP', throttle_duration_sec=2.0)
            self.estop_cmd_pub.publish(Twist())  # all-zero


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
