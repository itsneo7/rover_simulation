"""waypoint_manager node.

Sends a sequence of navigation goals to Nav2's NavigateToPose action server,
one at a time, waiting for each result before advancing. On failure it
retries (up to `max_retries`) before giving up on that waypoint and moving
on (configurable via `skip_on_failure`).

Waypoints are provided as a parameter (list of x,y,yaw triples) or can be
pushed programmatically by calling `set_waypoints()` before spinning.

    ros2 run ares1_navigation waypoint_manager --ros-args \
        -p waypoints:="[2.0,0.0,0.0, 5.0,1.0,1.57, 8.0,0.0,0.0]"
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


def yaw_to_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class WaypointManager(Node):

    def __init__(self):
        super().__init__('waypoint_manager')

        self.declare_parameter('waypoints', [0.0])   # flat [x0,y0,yaw0, x1,y1,yaw1, ...]
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('max_retries', 2)
        self.declare_parameter('skip_on_failure', True)
        self.declare_parameter('goal_timeout_sec', 120.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.max_retries = int(self.get_parameter('max_retries').value)
        self.skip_on_failure = bool(self.get_parameter('skip_on_failure').value)
        self.goal_timeout_sec = float(self.get_parameter('goal_timeout_sec').value)

        self.waypoints = self._parse_waypoints(self.get_parameter('waypoints').value)

        self._callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._callback_group)

        self._current_index = 0
        self._current_retries = 0
        self._goal_handle = None
        self._active = False

        self.get_logger().info(
            f'waypoint_manager loaded {len(self.waypoints)} waypoint(s).')

        if self.waypoints:
            self._start_timer = self.create_timer(1.0, self._kickoff_once)

    def set_waypoints(self, waypoints):
        """Programmatic API: waypoints is a list of (x, y, yaw) tuples."""
        self.waypoints = list(waypoints)

    @staticmethod
    def _parse_waypoints(flat_list):
        if not flat_list or len(flat_list) < 3:
            return []
        triples = []
        for i in range(0, len(flat_list) - 2, 3):
            triples.append((flat_list[i], flat_list[i + 1], flat_list[i + 2]))
        return triples

    def _kickoff_once(self):
        self._start_timer.cancel()
        self._send_next_waypoint()

    def _send_next_waypoint(self):
        if self._current_index >= len(self.waypoints):
            self.get_logger().info('All waypoints complete. Mission finished.')
            return

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server not available; retrying in 5s.')
            self.create_timer(5.0, self._send_next_waypoint)
            return

        x, y, yaw = self.waypoints[self._current_index]
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = yaw_to_quaternion(yaw)
        goal_msg.pose = pose

        self.get_logger().info(
            f'Sending waypoint {self._current_index + 1}/{len(self.waypoints)}: '
            f'x={x:.2f} y={y:.2f} yaw={yaw:.2f}')

        self._active = True
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _feedback_callback(self, feedback_msg):
        pass  # distance_remaining / navigation_time available here if needed

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Waypoint goal was rejected by Nav2.')
            self._handle_failure()
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        self._active = False
        status = future.result().status
        # status values follow action_msgs/GoalStatus (4 = SUCCEEDED)
        if status == 4:
            self.get_logger().info(
                f'Waypoint {self._current_index + 1} reached successfully.')
            self._current_index += 1
            self._current_retries = 0
            self._send_next_waypoint()
        else:
            self.get_logger().warn(
                f'Waypoint {self._current_index + 1} failed (status={status}).')
            self._handle_failure()

    def _handle_failure(self):
        if self._current_retries < self.max_retries:
            self._current_retries += 1
            self.get_logger().info(
                f'Retrying waypoint {self._current_index + 1} '
                f'(attempt {self._current_retries + 1}/{self.max_retries + 1}).')
            self._send_next_waypoint()
        elif self.skip_on_failure:
            self.get_logger().warn(
                f'Giving up on waypoint {self._current_index + 1} after '
                f'{self.max_retries} retries — skipping to next waypoint.')
            self._current_index += 1
            self._current_retries = 0
            self._send_next_waypoint()
        else:
            self.get_logger().error(
                f'Waypoint {self._current_index + 1} failed permanently. '
                'Mission aborted (skip_on_failure=false).')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
