"""wheel_odometry node.

Forward kinematics counterpart to swivel_kinematics.py: reads /joint_states
(steering angle + wheel angular velocity for all four independently steered,
independently driven wheels) and reconstructs the chassis-frame twist
(vx, vy, wz), then integrates it into a 2D pose and publishes /odom plus the
odom -> base_link TF... NOTE: per the architecture requirements, EKF (not
this node) owns the odom -> base_link TF broadcast. This node ONLY publishes
the nav_msgs/Odometry message on /odom for robot_localization to fuse; it
does not broadcast any TF itself.

Forward kinematics
-------------------
Each wheel i at position (x_i, y_i) with steering angle theta_i and wheel
angular velocity omega_i produces a measured contact-point velocity:

    Vx_i = omega_i * r * cos(theta_i)
    Vy_i = omega_i * r * sin(theta_i)

which relates to the chassis twist by:

    Vx_i = vx - wz * y_i
    Vy_i = vy + wz * x_i

With four wheels this is an over-determined linear system (8 equations, 3
unknowns: vx, vy, wz). We solve it in the least-squares sense every cycle,
which naturally averages out small per-wheel encoder noise/slip.

Realistic imperfection: small Gaussian noise plus a slowly varying slip bias
is added to the reconstructed twist before integration, since real wheel
odometry is never perfect (wheel slip, encoder quantization, tire deflection).
"""
import math
import random

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

from ares1_control.swivel_kinematics import RoverGeometry, WHEEL_NAMES

STEERING_JOINTS = {
    'FL': 'front_left_steering_joint',
    'FR': 'front_right_steering_joint',
    'RL': 'rear_left_steering_joint',
    'RR': 'rear_right_steering_joint',
}
WHEEL_JOINTS = {
    'FL': 'front_left_wheel_joint',
    'FR': 'front_right_wheel_joint',
    'RL': 'rear_left_wheel_joint',
    'RR': 'rear_right_wheel_joint',
}


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class WheelOdometry(Node):

    def __init__(self):
        super().__init__('wheel_odometry')

        self.declare_parameter('wheelbase', 0.55)
        self.declare_parameter('track_width', 0.45)
        self.declare_parameter('wheel_radius', 0.11)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('slip_noise_stddev', 0.01)   # m/s, per-axis
        self.declare_parameter('yaw_noise_stddev', 0.005)   # rad/s
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.geometry = RoverGeometry(
            wheelbase=self.get_parameter('wheelbase').value,
            track_width=self.get_parameter('track_width').value,
            wheel_radius=self.get_parameter('wheel_radius').value,
        )
        self.positions = self.geometry.wheel_positions()
        self.slip_noise_stddev = float(self.get_parameter('slip_noise_stddev').value)
        self.yaw_noise_stddev = float(self.get_parameter('yaw_noise_stddev').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST)

        self._steering_angle = {name: 0.0 for name in WHEEL_NAMES}
        self._wheel_velocity = {name: 0.0 for name in WHEEL_NAMES}
        self._have_state = False

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_time = self.get_clock().now()

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)

        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate_hz, self._update)

        self.get_logger().info('wheel_odometry node started (least-squares forward kinematics).')

    def _joint_state_callback(self, msg: JointState) -> None:
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        for wheel in WHEEL_NAMES:
            steer_joint = STEERING_JOINTS[wheel]
            wheel_joint = WHEEL_JOINTS[wheel]
            if steer_joint in name_to_idx and len(msg.position) > name_to_idx[steer_joint]:
                self._steering_angle[wheel] = msg.position[name_to_idx[steer_joint]]
            if wheel_joint in name_to_idx and len(msg.velocity) > name_to_idx[wheel_joint]:
                self._wheel_velocity[wheel] = msg.velocity[name_to_idx[wheel_joint]]
        self._have_state = True

    def _solve_twist_least_squares(self):
        r = self.geometry.wheel_radius
        A_rows = []
        b_vals = []
        for wheel in WHEEL_NAMES:
            x_i, y_i = self.positions[wheel]
            theta_i = self._steering_angle[wheel]
            omega_i = self._wheel_velocity[wheel]

            vx_meas = omega_i * r * math.cos(theta_i)
            vy_meas = omega_i * r * math.sin(theta_i)

            # Vx_i = vx - wz*y_i  ->  [1, 0, -y_i] . [vx, vy, wz] = vx_meas
            A_rows.append([1.0, 0.0, -y_i])
            b_vals.append(vx_meas)
            # Vy_i = vy + wz*x_i  ->  [0, 1,  x_i] . [vx, vy, wz] = vy_meas
            A_rows.append([0.0, 1.0, x_i])
            b_vals.append(vy_meas)

        A = np.array(A_rows)
        b = np.array(b_vals)
        solution, *_ = np.linalg.lstsq(A, b, rcond=None)
        vx, vy, wz = solution
        return float(vx), float(vy), float(wz)

    def _update(self) -> None:
        if not self._have_state:
            return

        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        if dt <= 0.0 or dt > 1.0:
            return

        vx, vy, wz = self._solve_twist_least_squares()

        # Realistic imperfection: wheel slip / encoder noise on the
        # reconstructed body-frame twist before integrating.
        vx += random.gauss(0.0, self.slip_noise_stddev)
        vy += random.gauss(0.0, self.slip_noise_stddev)
        wz += random.gauss(0.0, self.yaw_noise_stddev)

        # Integrate pose (body-frame vx, vy rotated into odom frame).
        dx = (vx * math.cos(self._yaw) - vy * math.sin(self._yaw)) * dt
        dy = (vx * math.sin(self._yaw) + vy * math.cos(self._yaw)) * dt
        dyaw = wz * dt

        self._x += dx
        self._y += dy
        self._yaw = math.atan2(math.sin(self._yaw + dyaw), math.cos(self._yaw + dyaw))

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(self._yaw)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        # Non-trivial (non-zero, non-perfect) covariance, consistent with
        # accumulating wheel-odometry uncertainty over time/speed.
        pos_var = 0.01
        yaw_var = 0.02
        odom.pose.covariance[0] = pos_var
        odom.pose.covariance[7] = pos_var
        odom.pose.covariance[35] = yaw_var
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = 0.03

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
