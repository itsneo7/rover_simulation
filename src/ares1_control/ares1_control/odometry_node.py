#!/usr/bin/env python3
"""ARES-1 Wheel Odometry Node."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math
import random

from ares1_control.swivel_kinematics import SwivelKinematics, SwivelConfig


class OdometryNode(Node):
    def __init__(self):
        super().__init__('ares1_odometry')

        self.declare_parameter('wheelbase', 0.55)
        self.declare_parameter('track_width', 0.45)
        self.declare_parameter('wheel_radius', 0.11)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('slip_factor', 0.02)
        self.declare_parameter('encoder_noise', 0.001)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        wb = self.get_parameter('wheelbase').value
        tw = self.get_parameter('track_width').value
        wr = self.get_parameter('wheel_radius').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.slip_factor = self.get_parameter('slip_factor').value
        self.encoder_noise = self.get_parameter('encoder_noise').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        config = SwivelConfig(
            wheelbase=wb, track_width=tw, wheel_radius=wr)
        self.kinematics = SwivelKinematics(config)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        self.get_logger().info('ARES-1 Odometry node started')

    def joint_state_callback(self, msg: JointState):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0 or dt > 0.5:
            self.last_time = current_time
            return

        joint_map = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position) and i < len(msg.velocity):
                joint_map[name] = {
                    'position': msg.position[i],
                    'velocity': msg.velocity[i],
                }

        steering_angles = {}
        wheel_speeds = {}

        wheel_names = ['front_left', 'front_right', 'rear_left', 'rear_right']
        keys = ['FL', 'FR', 'RL', 'RR']

        for key, prefix in zip(keys, wheel_names):
            steer_name = f'{prefix}_steering_joint'
            wheel_name = f'{prefix}_wheel_joint'

            if steer_name in joint_map:
                steering_angles[key] = joint_map[steer_name]['position']

            if wheel_name in joint_map:
                w_vel = joint_map[wheel_name]['velocity']
                wheel_speeds[key] = w_vel * self.kinematics.config.wheel_radius

        if not steering_angles or not wheel_speeds:
            return

        for key in wheel_speeds:
            noise = random.gauss(0, self.encoder_noise)
            slip = random.gauss(1.0, self.slip_factor)
            wheel_speeds[key] = wheel_speeds[key] * slip + noise

        vx, vy, wz = self.kinematics.forward_kinematics(
            steering_angles, wheel_speeds)

        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)

        self.x += (vx * cos_t - vy * sin_t) * dt
        self.y += (vx * sin_t + vy * cos_t) * dt
        self.theta += wz * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = self._yaw_to_quaternion(self.theta)
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.02
        odom.twist.covariance[0] = 0.005
        odom.twist.covariance[7] = 0.005
        odom.twist.covariance[35] = 0.01

        self.odom_pub.publish(odom)

        if self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

        self.last_time = current_time

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
