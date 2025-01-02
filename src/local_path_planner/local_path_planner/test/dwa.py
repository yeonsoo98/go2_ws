# local_path_planner/algorithms/dwa.py

import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion

class DWAAlgorithm:
    def __init__(self):
        pass  # 초기화가 필요한 경우 추가

    def compute_velocity(self, costmap: OccupancyGrid, odom: Odometry, goal: PoseStamped) -> Twist:
        cmd = Twist()

        # 현재 속도 추출
        current_v = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)
        current_w = odom.twist.twist.angular.z

        # 목표 위치와 로봇의 현재 위치 계산
        robot_x = odom.pose.pose.position.x
        robot_y = odom.pose.pose.position.y
        robot_yaw = self.get_yaw_from_quaternion(odom.pose.pose.orientation)

        goal_x = goal.pose.position.x
        goal_y = goal.pose.position.y

        # 단순한 P 제어기를 사용하여 선형 및 각속도 계산
        kp = 1.0
        ka = 4.0

        dx = goal_x - robot_x
        dy = goal_y - robot_y

        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - robot_yaw)

        # 선형 속도과 각속도 계산
        cmd.linear.x = kp * distance
        cmd.angular.z = ka * angle_diff

        # 속도 제한
        max_linear_speed = 1.0  # m/s
        max_angular_speed = 2.0  # rad/s

        cmd.linear.x = max(-max_linear_speed, min(max_linear_speed, cmd.linear.x))
        cmd.angular.z = max(-max_angular_speed, min(max_angular_speed, cmd.angular.z))

        # 장애물 회피 로직 추가 필요 (DWA의 핵심)
        # 여기서는 단순히 P 제어기를 사용한 예시입니다.

        return cmd

    def get_yaw_from_quaternion(self, q):
        roll, pitch, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        return yaw

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
