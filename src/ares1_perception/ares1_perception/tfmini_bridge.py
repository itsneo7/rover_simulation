"""tfmini_bridge node.

The TFmini-style forward range sensor is modeled in Gazebo as a single-ray
gpu_lidar (see ares1_description/urdf/sensors.xacro), which publishes a
1-sample sensor_msgs/LaserScan on /tfmini/scan_raw. This node converts that
into the semantically-correct sensor_msgs/Range message on /tfmini/range
that a real TFmini driver would publish.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, Range


class TFminiBridge(Node):

    def __init__(self):
        super().__init__('tfmini_bridge')

        self.declare_parameter('field_of_view', 0.05)  # rad, TFmini has a narrow beam
        self.declare_parameter('min_range', 0.10)
        self.declare_parameter('max_range', 12.0)
        self.declare_parameter('frame_id', 'tfmini_link')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)

        self.sub = self.create_subscription(
            LaserScan, '/tfmini/scan_raw', self._scan_callback, qos)
        self.pub = self.create_publisher(Range, '/tfmini/range', qos)

    def _scan_callback(self, msg: LaserScan) -> None:
        if not msg.ranges:
            return
        r = msg.ranges[0]

        out = Range()
        out.header = msg.header
        out.header.frame_id = self.get_parameter('frame_id').value
        out.radiation_type = Range.INFRARED
        out.field_of_view = float(self.get_parameter('field_of_view').value)
        out.min_range = float(self.get_parameter('min_range').value)
        out.max_range = float(self.get_parameter('max_range').value)
        out.range = r if math.isfinite(r) else out.max_range

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TFminiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
