#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class DWALocalPlanner(Node):
    def __init__(self):
        super().__init__('dwa_local_planner')

        # ROS 2 Publishers and Subscribers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/local_costmap', 10)

        # Parameters
        self.max_speed = 0.5  # Max linear velocity (m/s)
        self.max_yaw_rate = 1.0  # Max angular velocity (rad/s)
        self.v_res = 0.05  # Linear velocity resolution (m/s)
        self.w_res = 0.1  # Angular velocity resolution (rad/s)
        self.predict_time = 1.0  # Time to predict (s)
        self.robot_radius = 0.2  # Robot radius (m)
        self.goal = [2.0, 2.0]  # Goal position (x, y)

        # Costmap parameters
        self.costmap_size = 100  # Number of cells (square grid)
        self.resolution = 0.05  # Resolution of the grid (meters per cell)
        self.origin = [-2.5, -2.5]  # Origin of the costmap (meters)

        # State variables
        self.current_pose = None
        self.laser_ranges = []

    def odom_callback(self, msg):
        self.current_pose = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        ]

    def scan_callback(self, msg):
        self.laser_ranges = np.array(msg.ranges)
        self.update_costmap()

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        return math.atan2(siny_cosp, cosy_cosp)

    def update_costmap(self):
        # Initialize costmap as empty
        costmap = np.zeros((self.costmap_size, self.costmap_size), dtype=np.int8)

        for angle, dist in enumerate(self.laser_ranges):
            if 0.0 < dist < self.costmap_size * self.resolution:
                angle_rad = math.radians(angle)
                x = int((dist * math.cos(angle_rad) - self.origin[0]) / self.resolution)
                y = int((dist * math.sin(angle_rad) - self.origin[1]) / self.resolution)

                if 0 <= x < self.costmap_size and 0 <= y < self.costmap_size:
                    costmap[x, y] = 100  # Mark as obstacle

        # Publish the costmap
        costmap_msg = OccupancyGrid()
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.header.frame_id = 'base_link'
        costmap_msg.info.resolution = self.resolution
        costmap_msg.info.width = self.costmap_size
        costmap_msg.info.height = self.costmap_size
        costmap_msg.info.origin.position.x = self.origin[0]
        costmap_msg.info.origin.position.y = self.origin[1]
        costmap_msg.info.origin.position.z = 0.0
        costmap_msg.info.origin.orientation.w = 1.0
        costmap_msg.data = costmap.flatten().tolist()

        self.costmap_pub.publish(costmap_msg)

    def dwa_control(self):
        if self.current_pose is None or len(self.laser_ranges) == 0:
            return

        best_twist = Twist()
        best_cost = float('inf')

        for v in np.arange(0, self.max_speed, self.v_res):
            for w in np.arange(-self.max_yaw_rate, self.max_yaw_rate, self.w_res):
                cost = self.simulate_trajectory(v, w)
                if cost < best_cost:
                    best_cost = cost
                    best_twist.linear.x = v
                    best_twist.angular.z = w

        self.vel_pub.publish(best_twist)

    def simulate_trajectory(self, v, w):
        x, y, theta = self.current_pose
        cost = 0.0

        for t in np.arange(0, self.predict_time, 0.1):
            x += v * math.cos(theta) * 0.1
            y += v * math.sin(theta) * 0.1
            theta += w * 0.1

            # Calculate costs
            dist_to_goal = math.sqrt((self.goal[0] - x)**2 + (self.goal[1] - y)**2)
            obstacle_cost = self.check_obstacle_cost(x, y)

            if obstacle_cost == float('inf'):
                return float('inf')

            cost += dist_to_goal + obstacle_cost

        return cost

    def check_obstacle_cost(self, x, y):
        for angle, dist in enumerate(self.laser_ranges):
            if dist < self.robot_radius:
                return float('inf')

        return 0.0

    def main_loop(self):
        while rclpy.ok():
            self.dwa_control()
            rclpy.spin_once(self)


def main(args=None):
    rclpy.init(args=args)
    dwa_planner = DWALocalPlanner()
    dwa_planner.main_loop()
    dwa_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
