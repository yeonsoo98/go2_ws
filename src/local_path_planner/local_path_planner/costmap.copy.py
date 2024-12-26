#!/usr/bin/env python3

# 원본임 ㅇㅇ
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Bool
from tf2_ros import TransformListener, Buffer
from sensor_msgs_py import point_cloud2

import math

class CostmapGenerator(Node):
    def __init__(self):
        super().__init__('costmap_generator_copy')

        # ----- 파라미터 선언 -----
        # 로컬 코스트맵 사이즈 & 해상도
        self.declare_parameter('map_width', 100)
        self.declare_parameter('map_height', 100)
        self.declare_parameter('resolution', 0.05)

        # LIDAR, IMU, ODOM, BASE_FRAME
        self.declare_parameter('lidar_topic', '/velodyne_points')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('base_frame', 'base_link')

        # 업데이트 주기 & Inflation 반경 & Rolling window
        self.declare_parameter('update_frequency', 10.0)
        self.declare_parameter('inflation_radius', 0.1) # 벽 두께 설정
        self.declare_parameter('rolling_window', True)

        # 도착 범위 (예시): goal 근방 몇 m 이내면 도착으로 간주
        self.declare_parameter('arrival_threshold', 0.3)

        # ----- 파라미터 가져오기 -----
        self.map_width = self.get_parameter('map_width').get_parameter_value().integer_value
        self.map_height = self.get_parameter('map_height').get_parameter_value().integer_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value

        self.lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self.update_frequency = self.get_parameter('update_frequency').get_parameter_value().double_value
        self.inflation_radius = self.get_parameter('inflation_radius').get_parameter_value().double_value
        self.rolling_window = self.get_parameter('rolling_window').get_parameter_value().bool_value

        self.arrival_threshold = self.get_parameter('arrival_threshold').get_parameter_value().double_value

        # ----- TF 버퍼 & 리스너 -----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ----- OccupancyGrid 초기화 -----
        self.costmap = OccupancyGrid()
        self.costmap.header.frame_id = self.base_frame    ## 여기 수정
        self.costmap.info.resolution = self.resolution
        self.costmap.info.width = self.map_width
        self.costmap.info.height = self.map_height

        # 초기 origin 설정 (rolling_window=False 시, 고정)
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.reset_costmap_origin()

        # 초기 데이터 (모든 셀 0: free)
        self.costmap.data = [0]*(self.map_width*self.map_height)

        # ----- Subscriber -----
        self.lidar_sub = self.create_subscription(PointCloud2, self.lidar_topic, self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(Imu, self.imu_topic, self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        # (옵션) Goal Subscriber: 로봇이 goal 에 근접했는지 체크
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.current_goal = None

        # ----- Publisher -----
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/local_costmap', 10)
        # 로봇이 goal에 도착했는지 여부 (옵션)
        self.arrived_pub = self.create_publisher(Bool, '/arrived', 10)

        # ----- 내부 상태 -----
        self.current_imu = None
        self.current_odom = None
        self.lidar_points = []
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Timer
        self.timer = self.create_timer(1.0/self.update_frequency, self.publish_costmap)

    def reset_costmap_origin(self):
        """
        costmap.info.origin 설정
        """
        self.costmap.info.origin.position.x = self.origin_x
        self.costmap.info.origin.position.y = self.origin_y
        self.costmap.info.origin.position.z = 0.0
        self.costmap.info.origin.orientation = Quaternion(w=1.0)  # no rotation

    # ----- 콜백 함수들 -----
    def imu_callback(self, msg:Imu):
        self.current_imu = msg
        # pitch/roll 등 활용 가능

    def odom_callback(self, msg:Odometry):
        self.current_odom = msg
        # 로봇 위치 파악
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        # yaw
        q = msg.pose.pose.orientation
        # 간단 변환 (정확한 tf_transformations이나 transforms3d 사용)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # rolling window인 경우, costmap origin = (robot_x - width/2, robot_y - height/2)
        if self.rolling_window:
            half_w = self.map_width * self.resolution * 0.5
            half_h = self.map_height * self.resolution * 0.5
            self.origin_x = self.robot_x - half_w
            self.origin_y = self.robot_y - half_h
            self.reset_costmap_origin()

    def goal_callback(self, msg:PoseStamped):
        self.current_goal = msg

    def lidar_callback(self, msg:PointCloud2):
        points = []
        for p in point_cloud2.read_points(msg, field_names=("x","y","z"), skip_nans=True):
            points.append((p[0], p[1], p[2]))
        self.lidar_points = points

        # LIDAR 기반 OccupancyGrid 갱신
        self.update_costmap_from_points()

    # ----- 메인 costmap 업데이트 로직 -----
    def update_costmap_from_points(self):
        """
        LIDAR 포인트들 → costmap.data 업데이트
        1) 점을 costmap 좌표로 변환
        2) pz < threshold -> 장애물
        3) inflation_radius 반영
        """
        # 1) 맵 초기화 (0: free)
        data = [0]*(self.map_width*self.map_height)

        # 2) point to costmap
        for (px, py, pz) in self.lidar_points:
            if pz < 0.3:
                # 현재 costmap origin = (origin_x, origin_y)
                # 로봇 좌표계로 치면 rolling window 고려
                mx = int((px - self.costmap.info.origin.position.x)/self.resolution)
                my = int((py - self.costmap.info.origin.position.y)/self.resolution)
                if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                    idx = my * self.map_width + mx
                    data[idx] = 100

        # 3) Inflate
        inflate_cells = int(self.inflation_radius / self.resolution)
        inflated_data = data[:]  # copy
        for y in range(self.map_height):
            for x in range(self.map_width):
                idx = y*self.map_width + x
                if data[idx] == 100:  # obstacle
                    # 주변 inflate_cells에 대해서도 100
                    for dy in range(-inflate_cells, inflate_cells+1):
                        for dx2 in range(-inflate_cells, inflate_cells+1):
                            nx = x + dx2
                            ny = y + dy
                            if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                                distance = math.sqrt(dx2*dx2 + dy*dy) * self.resolution
                                if distance <= self.inflation_radius:
                                    n_idx = ny*self.map_width + nx
                                    inflated_data[n_idx] = 100

        self.costmap.data = inflated_data

    def publish_costmap(self):
        """
        1) costmap.header.stamp 갱신
        2) /local_costmap 퍼블리시
        3) (옵션) 로봇이 goal에 도착했는지 판단 → /arrived 퍼블리시
        """
        # 1) OccupancyGrid 헤더 갱신
        self.costmap.header.stamp = self.get_clock().now().to_msg()

        # 2) 퍼블리시
        self.costmap_pub.publish(self.costmap)

        # 3) 도착 판단 (옵션)
        if self.current_goal is not None and self.current_odom is not None:
            gx = self.current_goal.pose.position.x
            gy = self.current_goal.pose.position.y
            dist_to_goal = math.sqrt((gx - self.robot_x)**2 + (gy - self.robot_y)**2)
            if dist_to_goal < self.arrival_threshold:
                # 도착했으면 True
                arrived_msg = Bool(data=True)
                self.arrived_pub.publish(arrived_msg)
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
