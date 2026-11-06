"""obstacle_fusion node.

Associates each 2D detection (from object_detector, /perception/detections)
with depth at its bounding-box center (from the D456 depth image) to produce
a 3D obstacle position, using the standard pinhole back-projection:

    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    Z = depth(u, v)

computed in the camera optical frame, then transformed into base_link via
tf2. Results are published as a geometry_msgs/PoseArray on
/perception/object_positions for downstream consumption (e.g. semantic
obstacle markers, mission logic) and as a MarkerArray for RViz.

Note: the Nav2 local costmap's primary obstacle source is the geometric
point cloud from depth_processor (/perception/obstacle_points), which does
not depend on detection/classification at all — geometric obstacles are
detected and avoided even without a successful RGB detection. This node
adds semantic (classified, per-object) 3D positions on top of that, for
richer object-level bookkeeping.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_ros import TransformException
import tf_transformations
from message_filters import Subscriber, ApproximateTimeSynchronizer


class ObstacleFusion(Node):

    def __init__(self):
        super().__init__('obstacle_fusion')

        self.declare_parameter('depth_topic', '/realsense_D456/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/realsense_D456/depth/camera_info')
        self.declare_parameter('detections_topic', '/perception/detections')
        self.declare_parameter('base_frame', 'base_link')

        self.base_frame = self.get_parameter('base_frame').value
        self._camera_info = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)

        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._camera_info_callback, qos)

        self.depth_sub = Subscriber(
            self, Image, self.get_parameter('depth_topic').value,
            qos_profile=qos)
        self.det_sub = Subscriber(
            self, Detection2DArray, self.get_parameter('detections_topic').value,
            qos_profile=qos)

        self.sync = ApproximateTimeSynchronizer(
            [self.depth_sub, self.det_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self._synced_callback)

        self.pose_pub = self.create_publisher(PoseArray, '/perception/object_positions', qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/perception/object_markers', qos)

        self.get_logger().info('obstacle_fusion node started.')

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _synced_callback(self, depth_msg: Image, det_msg: Detection2DArray) -> None:
        if self._camera_info is None or not det_msg.detections:
            return

        fx = self._camera_info.k[0]
        fy = self._camera_info.k[4]
        cx_intr = self._camera_info.k[2]
        cy_intr = self._camera_info.k[5]

        depth_array = self._depth_msg_to_array(depth_msg)
        if depth_array is None:
            return

        transform = self._lookup_transform(depth_msg.header.frame_id)
        if transform is None:
            return

        poses = []
        markers = MarkerArray()
        marker_id = 0

        for det in det_msg.detections:
            u = int(round(det.bbox.center.position.x))
            v = int(round(det.bbox.center.position.y))
            if not (0 <= v < depth_array.shape[0] and 0 <= u < depth_array.shape[1]):
                continue
            z = float(depth_array[v, u])
            if not np.isfinite(z) or z <= 0.05 or z > 20.0:
                continue

            x = (u - cx_intr) * z / fx
            y = (v - cy_intr) * z / fy
            point_cam = np.array([x, y, z, 1.0])
            point_base = transform @ point_cam

            pose = Pose()
            pose.position.x = float(point_base[0])
            pose.position.y = float(point_base[1])
            pose.position.z = float(point_base[2])
            pose.orientation.w = 1.0
            poses.append(pose)

            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.header.stamp = depth_msg.header.stamp
            marker.ns = 'perception_objects'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker.lifetime.sec = 1
            markers.markers.append(marker)

        pose_array = PoseArray()
        pose_array.header.frame_id = self.base_frame
        pose_array.header.stamp = depth_msg.header.stamp
        pose_array.poses = poses

        self.pose_pub.publish(pose_array)
        self.marker_pub.publish(markers)

    def _lookup_transform(self, source_frame: str):
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

    @staticmethod
    def _depth_msg_to_array(msg: Image):
        if msg.encoding == '32FC1':
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            return arr
        elif msg.encoding == '16UC1':
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            return arr.astype(np.float32) / 1000.0  # mm -> m
        return None


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
