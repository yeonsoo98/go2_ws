#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion
from std_msgs.msg import Bool
from tf2_ros import TransformListener, Buffer
import math
from sensor_msgs_py import point_cloud2

class CostmapGenerator(Node):
    def __init__(self):
        super().__init__('costmap_generator')

        # ----- 파라미터 선언 -----
        self.declare_parameter('map_width', 100)
        self.declare_parameter('map_height', 100)
        self.declare_parameter('resolution', 0.05)

        self.declare_parameter('lidar_topic', '/velodyne_points')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('update_frequency', 10.0)
        self.declare_parameter('inflation_radius', 0.1)
        self.declare_parameter('rolling_window', True)
        self.declare_parameter('arrival_threshold', 0.3)

        # ----- 파라미터 가져오기 -----
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.resolution = self.get_parameter('resolution').value

        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.base_frame = self.get_parameter('base_frame').value

        self.update_frequency = self.get_parameter('update_frequency').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.rolling_window = self.get_parameter('rolling_window').value
        self.arrival_threshold = self.get_parameter('arrival_threshold').value

        # ----- TF 버퍼 & 리스너 -----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ----- OccupancyGrid 초기화 -----
        self.costmap = OccupancyGrid()
        self.costmap.header.frame_id = self.base_frame
        self.costmap.info.resolution = self.resolution
        self.costmap.info.width = self.map_width
        self.costmap.info.height = self.map_height

        # 초기 origin (rolling_window=False 시 고정)
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.reset_costmap_origin()

        # 초기 data (모두 free=0)
        self.costmap.data = [0]*(self.map_width*self.map_height)

        # ----- Subscriber -----
        self.lidar_sub = self.create_subscription(
            PointCloud2, self.lidar_topic, self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # ----- Publisher -----
        self.costmap_pub = self.create_publisher(
            OccupancyGrid, '/local_costmap', 10)
        self.arrived_pub = self.create_publisher(
            Bool, '/arrived', 10)

        # 내부 상태
        self.current_goal = None
        self.current_imu = None
        self.current_odom = None
        self.lidar_points = []
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # 주기적으로 costmap publish
        self.timer = self.create_timer(
            1.0/self.update_frequency, self.publish_costmap)

    def reset_costmap_origin(self):
        """costmap.info.origin 설정"""
        self.costmap.info.origin.position.x = self.origin_x
        self.costmap.info.origin.position.y = self.origin_y
        self.costmap.info.origin.position.z = 0.0
        self.costmap.info.origin.orientation = Quaternion(w=1.0)

    # -------------------- 콜백 함수들 --------------------
    def imu_callback(self, msg:Imu):
        # pitch/roll 활용 가능
        self.current_imu = msg

    def odom_callback(self, msg:Odometry):
        self.current_odom = msg
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # rolling window인 경우, 로봇 위치에 따라 origin 갱신
        if self.rolling_window:
            half_w = self.map_width * self.resolution * 0.5
            half_h = self.map_height * self.resolution * 0.5
            self.origin_x = self.robot_x - half_w
            self.origin_y = self.robot_y - half_h
            self.reset_costmap_origin()

    def goal_callback(self, msg:PoseStamped):
        self.current_goal = msg

    def lidar_callback(self, msg:PointCloud2):
        """
        LiDAR 포인트 (이미 base_link 기준이라고 가정)
        """
        new_points = []
        for p in point_cloud2.read_points(
                msg, field_names=("x","y","z"), skip_nans=True):
            new_points.append((p[0], p[1], p[2]))
        self.lidar_points = new_points

        # 업데이트
        self.update_costmap_from_points()

    # -------------------- 메인 costmap 업데이트 로직 --------------------
    def update_costmap_from_points(self):
        """
        장애물 판정
        - z 범위: [0.0, 1.5]   (필요에 맞게 조정)
        - close_threshold: 0.05m (코앞)
        - 360도 어느 방향이든 인식 (px>0 제거)
        """
        data = [0]*(self.map_width*self.map_height)

        min_z = 0.0
        max_z = 1.5
        close_threshold = 0.01  # 로봇 기준 5cm 근접

        for (px, py, pz) in self.lidar_points:
            # 로봇에서의 거리
            dist_robot = math.hypot(px, py)
            if dist_robot < close_threshold:
                # 너무 가깝다: 장애물
                obstacle = True
            else:
                # z 범위 내면 장애물
                if min_z < pz < max_z:
                    obstacle = True
                else:
                    obstacle = False

            if obstacle:
                # base_link coords → costmap index
                mx = int((px - self.costmap.info.origin.position.x)
                         / self.resolution)
                my = int((py - self.costmap.info.origin.position.y)
                         / self.resolution)

                if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                    idx = my*self.map_width + mx
                    data[idx] = 100

        # 인플레이션
        inflate_cells = int(self.inflation_radius / self.resolution)
        inflated_data = data[:]
        for y in range(self.map_height):
            for x in range(self.map_width):
                idx = y*self.map_width + x
                if data[idx] == 100:
                    for dy in range(-inflate_cells, inflate_cells+1):
                        for dx2 in range(-inflate_cells, inflate_cells+1):
                            nx = x + dx2
                            ny = y + dy
                            if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                                dist = math.sqrt(dx2*dx2 + dy*dy)*self.resolution
                                if dist <= self.inflation_radius:
                                    n_idx = ny*self.map_width + nx
                                    inflated_data[n_idx] = 100

        self.costmap.data = inflated_data

    def publish_costmap(self):
        # 헤더 stamp 갱신 
        self.costmap.header.stamp = self.get_clock().now().to_msg()
        self.costmap_pub.publish(self.costmap)

        # goal 도착 판단
        if self.current_goal and self.current_odom:
            gx = self.current_goal.pose.position.x
            gy = self.current_goal.pose.position.y
            dist = math.hypot(gx - self.robot_x, gy - self.robot_y)
            if dist < self.arrival_threshold:
                arrived_msg = Bool(data=True)
            else:
                arrived_msg = Bool(data=False)
            self.arrived_pub.publish(arrived_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CostmapGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
