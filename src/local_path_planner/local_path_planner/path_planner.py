#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from local_path_planner.algorithms.dwa import DWAAlgorithm
from local_path_planner.algorithms.teb import TEBAlgorithm
from tf_transformations import euler_from_quaternion
import math

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # ---- 파라미터 ----
        self.declare_parameter('algorithm_name', 'dwa')
        self.declare_parameter('goal_arrival_tolerance', 0.2)
        self.declare_parameter('plan_loop_rate', 10.0)  # Hz

        self.algorithm_name = self.get_parameter('algorithm_name').get_parameter_value().string_value
        self.goal_arrival_tolerance = self.get_parameter('goal_arrival_tolerance').get_parameter_value().double_value
        self.plan_loop_rate = self.get_parameter('plan_loop_rate').get_parameter_value().double_value

        # ---- 알고리즘 선택 ----
        if self.algorithm_name == 'dwa':
            self.planner = DWAAlgorithm()
        else:
            self.planner = TEBAlgorithm()

        # ---- Subscriber ----
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap', self.costmap_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom/ground_truth', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # ---- Publisher ----
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---- 내부 상태 ----
        self.current_costmap = None
        self.current_odom = None
        self.current_goal = None
        self.robot_yaw = 0.0
        self.arrived = False

        # ---- Timer (plan_loop_rate) ----
        timer_period = 1.0 / self.plan_loop_rate
        self.timer = self.create_timer(timer_period, self.update_cmd_vel)

    def costmap_callback(self, msg:OccupancyGrid):
        self.current_costmap = msg

    def odom_callback(self, msg:Odometry):
        self.current_odom = msg
        # 로봇 Yaw
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.robot_yaw = yaw

    def goal_callback(self, msg:PoseStamped):
        self.current_goal = msg
        self.arrived = False
        self.get_logger().info("New goal received: (%.2f, %.2f)" % 
            (msg.pose.position.x, msg.pose.position.y))

    # 여기 수정
    def update_cmd_vel(self):
        # 1) goal, costmap, odom 미존재 시 정지
        if not self.current_goal or not self.current_costmap or not self.current_odom:
            self.publish_stop()
            return

        # 2) 목표 도달 확인
        if self.is_goal_reached():
            if not self.arrived:
                self.get_logger().info("Goal reached! Stopping robot.")
            self.arrived = True
            self.publish_stop()
            return

        # 3) DWA 알고리즘 실행
        cmd = self.planner.compute_velocity(
            costmap=self.current_costmap,
            odom=self.current_odom,
            goal=self.current_goal
        )
        self.cmd_vel_pub.publish(cmd)

    def is_goal_reached(self):
        gx, gy = self.current_goal.pose.position.x, self.current_goal.pose.position.y
        rx, ry = self.current_odom.pose.pose.position.x, self.current_odom.pose.pose.position.y
        dist = math.hypot(gx - rx, gy - ry)
        return dist < self.goal_arrival_tolerance

    def publish_stop(self):
        """
        cmd_vel = 0
        """
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()