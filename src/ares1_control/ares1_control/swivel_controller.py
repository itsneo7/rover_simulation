"""swivel_controller node.

Subscribes to /cmd_vel (geometry_msgs/Twist, produced by nav2_mppi_controller
with the Omni motion model: linear.x = vx, linear.y = vy, angular.z = wz).

Runs the ARES-1 swivel inverse kinematics every cycle and publishes:

  - /steering_controller/commands   (std_msgs/Float64MultiArray)
      order: [FL, FR, RL, RR] steering position, radians
  - /wheel_velocity_controller/commands (std_msgs/Float64MultiArray)
      order: [FL, FR, RL, RR] wheel angular velocity, rad/s

matching the `forward_position_controller` / `forward_velocity_controller`
(ros2_controllers "forward_command_controller") topics declared in
config/ros2_controllers.yaml.

Includes a command-timeout watchdog: if no /cmd_vel is received within
`cmd_timeout_sec`, wheel velocities are zeroed (steering angles held) so the
rover does not run away on a dropped connection.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Bool

from ares1_control.swivel_kinematics import RoverGeometry, SwivelKinematics, WHEEL_NAMES


class SwivelController(Node):

    def __init__(self):
        super().__init__('swivel_controller')

        self.declare_parameter('wheelbase', 0.55)
        self.declare_parameter('track_width', 0.45)
        self.declare_parameter('wheel_radius', 0.11)
        self.declare_parameter('max_steering_angle', math.pi)
        self.declare_parameter('max_linear_speed', 1.5)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)

        geometry = RoverGeometry(
            wheelbase=self.get_parameter('wheelbase').value,
            track_width=self.get_parameter('track_width').value,
            wheel_radius=self.get_parameter('wheel_radius').value,
        )
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)

        self.kinematics = SwivelKinematics(
            geometry=geometry,
            max_steering_angle=float(self.get_parameter('max_steering_angle').value),
        )

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)

        self._latest_cmd = Twist()
        self._last_cmd_time = self.get_clock().now()
        self._have_received_cmd = False
        self._estop_active = False

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, qos)
        self.estop_sub = self.create_subscription(
            Bool, '/safety/estop', self._estop_callback, qos)

        self.steering_pub = self.create_publisher(
            Float64MultiArray, '/steering_controller/commands', qos)
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/wheel_velocity_controller/commands', qos)

        rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate_hz, self._control_step)

        self.get_logger().info(
            f'swivel_controller started: wheelbase={geometry.wheelbase} '
            f'track_width={geometry.track_width} wheel_radius={geometry.wheel_radius}')

    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._latest_cmd = msg
        self._last_cmd_time = self.get_clock().now()
        self._have_received_cmd = True

    def _estop_callback(self, msg: Bool) -> None:
        if msg.data and not self._estop_active:
            self.get_logger().error('EMERGENCY STOP received from safety_watchdog — halting rover.')
        self._estop_active = bool(msg.data)

    def _control_step(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self._last_cmd_time).nanoseconds * 1e-9

        if self._estop_active:
            vx, vy, wz = 0.0, 0.0, 0.0
        elif self._have_received_cmd and elapsed <= self.cmd_timeout_sec:
            vx = self._clamp(self._latest_cmd.linear.x, self.max_linear_speed)
            vy = self._clamp(self._latest_cmd.linear.y, self.max_linear_speed)
            wz = self._clamp(self._latest_cmd.angular.z, self.max_angular_speed)
        else:
            # Watchdog: command timed out or nothing received yet -> stop.
            vx, vy, wz = 0.0, 0.0, 0.0
            if self._have_received_cmd and elapsed > self.cmd_timeout_sec:
                self.get_logger().warn(
                    f'/cmd_vel timeout ({elapsed:.2f}s) — commanding zero velocity.',
                    throttle_duration_sec=2.0,
                )

        commands = self.kinematics.compute(vx, vy, wz)

        steering_msg = Float64MultiArray()
        wheel_msg = Float64MultiArray()
        steering_msg.data = [commands[name].steering_angle for name in WHEEL_NAMES]
        wheel_msg.data = [commands[name].wheel_angular_velocity for name in WHEEL_NAMES]

        self.steering_pub.publish(steering_msg)
        self.wheel_pub.publish(wheel_msg)

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))


def main(args=None):
    rclpy.init(args=args)
    node = SwivelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
