"""depth_processor node.

Pipeline (per section 19 of the ARES-1 spec):

  D456 depth -> point cloud -> ground filtering -> obstacle extraction
  -> 3D obstacle information -> Nav2 costmap (obstacle/pointcloud layer)

Subscribes:
  /realsense_D456/depth/points   (sensor_msgs/PointCloud2, camera optical frame)

Publishes:
  /perception/obstacle_points    (sensor_msgs/PointCloud2, base_link frame)
      Points classified as NOT ground (used by the Nav2 local costmap
      pointcloud obstacle layer).
  /perception/ground_points      (sensor_msgs/PointCloud2, base_link frame)
      Points classified as ground (for debugging/visualization only).

Ground filtering method:
  Each point is transformed from the camera optical frame into base_link
  using tf2 (the transform is static/fixed since the D456 is rigidly
  mounted). A point is classified as "ground" if its base_link-frame Z is
  within `ground_z_threshold` of 0.0 (the wheel-contact plane); everything
  else — low obstacles, raised objects, terrain bumps, drop-off rims — is
  treated as an obstacle candidate. A minimum-height filter additionally
  discards points far above the rover (e.g. tree canopy) that would
  otherwise pollute the local costmap.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
import tf2_ros
from tf2_ros import TransformException
import tf_transformations


class DepthProcessor(Node):

    def __init__(self):
        super().__init__('depth_processor')

        self.declare_parameter('input_topic', '/realsense_D456/depth/points')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ground_z_threshold', 0.06)   # meters
        self.declare_parameter('max_obstacle_height', 1.5)   # meters
        self.declare_parameter('min_obstacle_height', -0.5)  # meters (drop-offs)
        self.declare_parameter('max_range', 8.0)              # meters

        self.input_topic = self.get_parameter('input_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.ground_z_threshold = float(self.get_parameter('ground_z_threshold').value)
        self.max_obstacle_height = float(self.get_parameter('max_obstacle_height').value)
        self.min_obstacle_height = float(self.get_parameter('min_obstacle_height').value)
        self.max_range = float(self.get_parameter('max_range').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)

        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self._cloud_callback, qos)
        self.obstacle_pub = self.create_publisher(
            PointCloud2, '/perception/obstacle_points', qos)
        self.ground_pub = self.create_publisher(
            PointCloud2, '/perception/ground_points', qos)

        self.get_logger().info(f'depth_processor listening on {self.input_topic}')

    def _lookup_static_transform_matrix(self, source_frame: str):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, source_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(
                f'Could not transform {source_frame} -> {self.base_frame}: {ex}',
                throttle_duration_sec=5.0)
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        matrix = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        matrix[0, 3] = t.x
        matrix[1, 3] = t.y
        matrix[2, 3] = t.z
        return matrix

    def _cloud_callback(self, msg: PointCloud2) -> None:
        transform = self._lookup_static_transform_matrix(msg.header.frame_id)
        if transform is None:
            return

        points = np.array(list(pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)))
        if points.size == 0:
            return

        # points is a structured array with named fields when skip_nans;
        # normalize into a plain (N,3) float array.
        pts = np.vstack([points['x'], points['y'], points['z']]).T.astype(np.float64)

        # Range gate in the original (camera) frame before transforming.
        ranges = np.linalg.norm(pts, axis=1)
        pts = pts[ranges <= self.max_range]
        if pts.shape[0] == 0:
            return

        ones = np.ones((pts.shape[0], 1))
        pts_h = np.hstack([pts, ones])
        pts_base = (transform @ pts_h.T).T[:, :3]

        z = pts_base[:, 2]
        height_ok = (z > self.min_obstacle_height) & (z < self.max_obstacle_height)
        pts_base = pts_base[height_ok]
        z = z[height_ok]

        is_ground = np.abs(z) <= self.ground_z_threshold
        ground_pts = pts_base[is_ground]
        obstacle_pts = pts_base[~is_ground]

        self._publish_cloud(self.obstacle_pub, obstacle_pts, msg.header.stamp)
        self._publish_cloud(self.ground_pub, ground_pts, msg.header.stamp)

    def _publish_cloud(self, publisher, points: np.ndarray, stamp) -> None:
        header = rclpy.time.Time()
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg = pc2.create_cloud(
            header=self._make_header(stamp), fields=fields,
            points=points.astype(np.float32).tolist())
        publisher.publish(cloud_msg)

    def _make_header(self, stamp):
        from std_msgs.msg import Header
        h = Header()
        h.stamp = stamp
        h.frame_id = self.base_frame
        return h


def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
