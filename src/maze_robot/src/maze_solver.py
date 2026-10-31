#!/usr/bin/env python3
import math
import heapq
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class FastMazeExplorerAndAStarSolver(Node):
    def __init__(self):
        super().__init__('maze_solver')

        # Robot spawns at (1.0, 11.0) in Gazebo, which is (0.0, 0.0) in map/odom frame.
        # Goal Pad is at (11.0, 1.0) in Gazebo -> (+10.0, -10.0) in map/odom frame!
        self.start_xy = (0.0, 0.0)
        self.goal_xy = (10.0, -10.0)

        # Maze outer bounds in map frame: X in [-1.0, 11.0], Y in [-11.0, 1.0]
        self.res = 0.10  # 10cm A* planning resolution (120 x 120 cells)
        self.grid_w = 120
        self.grid_h = 120
        self.origin_x = -1.0
        self.origin_y = -11.0

        # Persistent obstacle memory so A* remembers every wall seen by Lidar + SLAM
        self.master_occ = np.zeros((self.grid_h, self.grid_w), dtype=bool)
        # Lock outer boundary walls so A* NEVER routes outside the 12x12m maze
        self.master_occ[0:3, :] = True
        self.master_occ[-3:, :] = True
        self.master_occ[:, 0:3] = True
        self.master_occ[:, -3:] = True

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.scan_msg = None
        self.map_msg = None

        self.phase = 'EXPLORE_AND_SOLVE'
        self.planned_path = []
        self.last_astar_time = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/solver_markers', 10)

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('🚀 Open-Labyrinth Explorer + Bounded A* Solver Ready!')

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.res)
        gy = int((y - self.origin_y) / self.res)
        return max(0, min(self.grid_w - 1, gx)), max(0, min(self.grid_h - 1, gy))

    def grid_to_world(self, gx, gy):
        return self.origin_x + (gx + 0.5) * self.res, self.origin_y + (gy + 0.5) * self.res

    def odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def scan_cb(self, msg: LaserScan):
        self.scan_msg = msg
        # Project live Lidar hits directly into master_occ for instant wall awareness
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges, dtype=np.float32)
        valid = (ranges > 0.15) & (ranges < 8.0) & np.isfinite(ranges)
        hit_x = self.robot_x + ranges[valid] * np.cos(self.robot_yaw + angles[valid])
        hit_y = self.robot_y + ranges[valid] * np.sin(self.robot_yaw + angles[valid])
        for hx, hy in zip(hit_x[::2], hit_y[::2]):
            gx, gy = self.world_to_grid(hx, hy)
            self.master_occ[gy, gx] = True

    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg
        # Sync SLAM Toolbox map walls into our bounded 12x12m master grid
        info = msg.info
        raw = np.array(msg.data, dtype=np.int8).reshape((info.height, info.width))
        occ_y, occ_x = np.where(raw > 55)
        for r, c in zip(occ_y[::2], occ_x[::2]):
            wx = info.origin.position.x + (c + 0.5) * info.resolution
            wy = info.origin.position.y + (r + 0.5) * info.resolution
            gx, gy = self.world_to_grid(wx, wy)
            self.master_occ[gy, gx] = True

    def run_astar(self, start_xy, target_xy):
        # Inflate walls by 4 cells (0.40m) so path stays dead-center in the 2.0m corridors
        inflated = self.master_occ.copy()
        for dx in range(-4, 5):
            for dy in range(-4, 5):
                if dx * dx + dy * dy <= 16:
                    inflated |= np.roll(np.roll(self.master_occ, dy, axis=0), dx, axis=1)

        sx, sy = self.world_to_grid(start_xy[0], start_xy[1])
        gx, gy = self.world_to_grid(target_xy[0], target_xy[1])

        # Ensure robot's immediate cell is free
        inflated[max(0, sy - 3):min(self.grid_h, sy + 4), max(0, sx - 3):min(self.grid_w, sx + 4)] = False
        inflated[max(0, gy - 3):min(self.grid_h, gy + 4), max(0, gx - 3):min(self.grid_w, gx + 4)] = False

        start_node = (sx, sy)
        goal_node = (gx, gy)

        open_set = [(0.0, start_node)]
        came_from = {}
        g_score = {start_node: 0.0}
        neighbors = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                     (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]

        while open_set:
            _, current = heapq.heappop(open_set)
            if math.hypot(current[0] - goal_node[0], current[1] - goal_node[1]) <= 1.5:
                path = []
                while current in came_from:
                    path.append(self.grid_to_world(current[0], current[1]))
                    current = came_from[current]
                path.reverse()
                path.append(target_xy)
                return path

            for dx, dy, cost in neighbors:
                nx, ny = current[0] + dx, current[1] + dy
                if 2 <= nx < self.grid_w - 2 and 2 <= ny < self.grid_h - 2 and not inflated[ny, nx]:
                    tg = g_score[current] + cost
                    if tg < g_score.get((nx, ny), 1e9):
                        came_from[(nx, ny)] = current
                        g_score[(nx, ny)] = tg
                        f = tg + math.hypot(nx - goal_node[0], ny - goal_node[1])
                        heapq.heappush(open_set, (f, (nx, ny)))
        return []

    def publish_visuals(self):
        now = self.get_clock().now().to_msg()
        if self.planned_path:
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = now
            for px, py in self.planned_path:
                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x = float(px)
                ps.pose.position.y = float(py)
                ps.pose.orientation.w = 1.0
                path_msg.poses.append(ps)
            self.path_pub.publish(path_msg)

        ma = MarkerArray()
        gm = Marker()
        gm.header.frame_id = 'map'
        gm.header.stamp = now
        gm.ns = 'goal_target'
        gm.id = 0
        gm.type = Marker.CYLINDER
        gm.action = Marker.ADD
        gm.pose.position.x = float(self.goal_xy[0])
        gm.pose.position.y = float(self.goal_xy[1])
        gm.pose.position.z = 0.3
        gm.scale.x = 0.8
        gm.scale.y = 0.8
        gm.scale.z = 0.6
        gm.color.r = 0.1
        gm.color.g = 0.95
        gm.color.b = 0.2
        gm.color.a = 0.85
        ma.markers.append(gm)
        self.marker_pub.publish(ma)

    def follow_path(self, max_speed):
        if self.scan_msg is not None:
            ranges = np.array(self.scan_msg.ranges, dtype=np.float32)
            ranges = np.where(np.isinf(ranges) | np.isnan(ranges), 10.0, ranges)
            mid = len(ranges) // 2
            front = float(np.min(ranges[mid - 20:mid + 20]))
            left = float(np.mean(ranges[mid + 20:mid + 90]))
            right = float(np.mean(ranges[mid - 90:mid - 20]))
            if front < 0.42:
                return 0.0, (1.1 if left > right else -1.1)

        if not self.planned_path:
            return 0.4, 0.0

        # Pick waypoint ~0.75m ahead along the A* curve
        target_pt = self.planned_path[-1]
        for px, py in self.planned_path:
            if math.hypot(px - self.robot_x, py - self.robot_y) >= 0.75:
                target_pt = (px, py)
                break

        dx = target_pt[0] - self.robot_x
        dy = target_pt[1] - self.robot_y
        target_yaw = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(target_yaw - self.robot_yaw), math.cos(target_yaw - self.robot_yaw))

        if abs(yaw_err) > 0.7:
            return 0.08, (1.2 if yaw_err > 0 else -1.2)

        v = max_speed * max(0.3, 1.0 - 0.6 * abs(yaw_err))
        w = 1.8 * yaw_err
        return v, w

    def control_loop(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # Recompute A* every 0.4 seconds as Lidar uncovers walls
        if now_sec - self.last_astar_time > 0.4:
            self.last_astar_time = now_sec
            target = self.start_xy if self.phase == 'RETURN_TO_START' else self.goal_xy
            self.planned_path = self.run_astar((self.robot_x, self.robot_y), target)

        self.publish_visuals()

        twist = Twist()
        if self.phase == 'EXPLORE_AND_SOLVE':
            if math.hypot(self.goal_xy[0] - self.robot_x, self.goal_xy[1] - self.robot_y) < 0.65:
                self.get_logger().info('🏆 PHASE 1 COMPLETE: Maze Solved & Mapped! Returning to Blue Start Pad...')
                self.phase = 'RETURN_TO_START'
            else:
                v, w = self.follow_path(max_speed=0.60)
                twist.linear.x, twist.angular.z = float(v), float(w)

        elif self.phase == 'RETURN_TO_START':
            if math.hypot(self.start_xy[0] - self.robot_x, self.start_xy[1] - self.robot_y) < 0.65:
                self.get_logger().info('🚀 BACK AT START! Executing Maximum-Speed Known-Map A* Sprint!')
                self.phase = 'SPEED_RUN_ASTAR'
            else:
                v, w = self.follow_path(max_speed=0.80)
                twist.linear.x, twist.angular.z = float(v), float(w)

        elif self.phase == 'SPEED_RUN_ASTAR':
            if math.hypot(self.goal_xy[0] - self.robot_x, self.goal_xy[1] - self.robot_y) < 0.65:
                self.get_logger().info('🏁 HIGH-SPEED A* SPRINT COMPLETE! Parked on Green Goal Pad.')
                self.phase = 'DONE'
            else:
                v, w = self.follow_path(max_speed=1.05)
                twist.linear.x, twist.angular.z = float(v), float(w)

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FastMazeExplorerAndAStarSolver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
