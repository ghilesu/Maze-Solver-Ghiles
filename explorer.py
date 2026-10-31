#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class MazeExplorer(Node):
    def __init__(self):
        super().__init__('maze_explorer')
        # Publish to wheels, Subscribe to LiDAR
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd = Twist()

    def scan_callback(self, msg):
        # Gazebo LiDAR: 360 samples. Index 180 is straight ahead, 270 is left, 90 is right.
        front_scan = msg.ranges[160:200]
        left_scan = msg.ranges[250:290]
        right_scan = msg.ranges[70:110]

        # Filter out invalid LiDAR bounces (zeros or infinities)
        valid_front = [d for d in front_scan if not math.isinf(d) and not math.isnan(d) and d > 0.0]
        valid_left = [d for d in left_scan if not math.isinf(d) and not math.isnan(d) and d > 0.0]
        valid_right = [d for d in right_scan if not math.isinf(d) and not math.isnan(d) and d > 0.0]

        # Get minimum distance in each direction (default to 10m if empty)
        front_dist = min(valid_front) if valid_front else 10.0
        left_dist = min(valid_left) if valid_left else 10.0
        right_dist = min(valid_right) if valid_right else 10.0

        # Logic: If front is clear, drive straight. Else, turn to the most open side.
        if front_dist > 0.65:
            self.cmd.linear.x = 0.25   # Drive forward at 0.25 m/s
            self.cmd.angular.z = 0.0
        else:
            self.cmd.linear.x = 0.0    # Stop moving forward
            if left_dist > right_dist:
                self.cmd.angular.z = 0.4   # Turn Left
            else:
                self.cmd.angular.z = -0.4  # Turn Right

        self.pub.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MazeExplorer()
    print("Autonomous Exploration Started! Watch the robot in Gazebo.")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
