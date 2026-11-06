#!/usr/bin/env python3
"""ARES-1 Custom A* Global Planner."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import heapq
import math


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('a_star_planner')

        self.declare_parameter('obstacle_threshold', 50)
        self.declare_parameter('allow_unknown', False)

        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.allow_unknown = self.get_parameter('allow_unknown').value

        self.costmap = None
        self.costmap_info = None
        self.current_pose = None

        costmap_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.path_pub = self.create_publisher(Path, '/global_plan', 10)

        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            self.costmap_callback, costmap_qos)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/odometry/filtered',
            self.pose_callback, 10)

        self.get_logger().info('A* Global Planner started')

    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap = msg.data
        self.costmap_info = msg.info

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg: PoseStamped):
        if self.costmap is None or self.costmap_info is None:
            self.get_logger().warn('No costmap received yet')
            return

        if self.current_pose is None:
            self.get_logger().warn('No current pose received yet')
            return

        start = self._world_to_grid(
            self.current_pose.position.x,
            self.current_pose.position.y)
        goal = self._world_to_grid(
            msg.pose.position.x,
            msg.pose.position.y)

        if start is None or goal is None:
            self.get_logger().error('Start or goal outside costmap')
            return

        self.get_logger().info(
            f'Planning A* from {start} to {goal}...')

        path_grid = self._a_star(start, goal)

        if path_grid is None:
            self.get_logger().warn('A* failed to find a path')
            return

        path = Path()
        path.header = msg.header

        for gx, gy in path_grid:
            wx, wy = self._grid_to_world(gx, gy)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.path_pub.publish(path)
        self.get_logger().info(
            f'A* path found with {len(path.poses)} waypoints')

    def _world_to_grid(self, wx, wy):
        if self.costmap_info is None:
            return None
        resolution = self.costmap_info.resolution
        origin_x = self.costmap_info.origin.position.x
        origin_y = self.costmap_info.origin.position.y
        width = self.costmap_info.width
        height = self.costmap_info.height
        gx = int((wx - origin_x) / resolution)
        gy = int((wy - origin_y) / resolution)
        if 0 <= gx < width and 0 <= gy < height:
            return (gx, gy)
        return None

    def _grid_to_world(self, gx, gy):
        resolution = self.costmap_info.resolution
        origin_x = self.costmap_info.origin.position.x
        origin_y = self.costmap_info.origin.position.y
        wx = origin_x + (gx + 0.5) * resolution
        wy = origin_y + (gy + 0.5) * resolution
        return (wx, wy)

    def _is_occupied(self, gx, gy):
        width = self.costmap_info.width
        idx = gy * width + gx
        if idx < 0 or idx >= len(self.costmap):
            return True
        val = self.costmap[idx]
        if val == -1:
            return not self.allow_unknown
        return val >= self.obstacle_threshold

    def _a_star(self, start, goal):
        width = self.costmap_info.width
        height = self.costmap_info.height

        def heuristic(a, b):
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

        neighbors = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414),
            (1, -1, 1.414), (1, 1, 1.414),
        ]

        open_set = [(heuristic(start, goal), 0.0, start, None)]
        visited = set()
        came_from = {}
        g_scores = {start: 0.0}

        while open_set:
            _, g, current, parent = heapq.heappop(open_set)

            if current in visited:
                continue
            visited.add(current)
            came_from[current] = parent

            if current == goal:
                path = []
                c = current
                while c is not None:
                    path.append(c)
                    c = came_from[c]
                path.reverse()
                return path

            cx, cy = current
            for dx, dy, cost in neighbors:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < width and 0 <= ny < height):
                    continue
                if (nx, ny) in visited:
                    continue
                if self._is_occupied(nx, ny):
                    continue
                ng = g + cost
                if (nx, ny) not in g_scores or ng < g_scores[(nx, ny)]:
                    g_scores[(nx, ny)] = ng
                    f = ng + heuristic((nx, ny), goal)
                    heapq.heappush(
                        open_set, (f, ng, (nx, ny), current))

        return None


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
