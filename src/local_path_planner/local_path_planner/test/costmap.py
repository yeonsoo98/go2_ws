#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Quaternion, Pose
from std_msgs.msg import Bool
from tf_transformations import euler_from_quaternion
from tf2_ros import TransformListener, Buffer
import tf2_ros
import math

# PointCloud2 helpers
from sensor_msgs_py.point_cloud2 import read_points, create_cloud_xyz32
import PyKDL


class CostmapGenerator(Node):
    def __init__(self):
        super().__init__('costmap_generator')

        # ------------------- Parameters -------------------
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.declare_parameter('map_width', 100)
        self.declare_parameter('map_height', 100)
        self.declare_parameter('resolution', 0.05)

        self.declare_parameter('lidar_topic', '/velodyne_points')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', 'odom/ground_truth')  # ground_truth
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('update_frequency', 10.0)
        self.declare_parameter('inflation_radius', 0.1)
        self.declare_parameter('rolling_window', True)
        self.declare_parameter('arrival_threshold', 0.3)

        # ------------------- Retrieve Parameters -------------------
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

        # ------------------- TF Buffer & Listener -------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ------------------- OccupancyGrid Initialization -------------------
        self.costmap = OccupancyGrid()
        self.costmap.header.frame_id = self.base_frame
        self.costmap.info.resolution = self.resolution
        self.costmap.info.width = self.map_width
        self.costmap.info.height = self.map_height

        self.origin_x = 0.0
        self.origin_y = 0.0
        self.reset_costmap_origin()

        self.costmap.data = [0] * (self.map_width * self.map_height)

        # ------------------- Subscribers -------------------
        self.lidar_sub = self.create_subscription(
            PointCloud2, self.lidar_topic, self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, self.imu_topic, self.imu_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # ------------------- Publishers -------------------
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/local_costmap', 10)
        self.arrived_pub = self.create_publisher(Bool, '/arrived', 10)

        self.transformed_cloud_pub = self.create_publisher(PointCloud2, '/transformed_points', 10)
        self.transformed_odometry_pub = self.create_publisher(Odometry, '/transformed_odometry', 10)  # New Publisher

        # Internal state
        self.current_goal = None
        self.current_imu = None
        self.current_odom = None
        self.lidar_points = []

        # Timer for costmap publishing
        self.timer = self.create_timer(1.0 / self.update_frequency, self.publish_costmap)

    def reset_costmap_origin(self):
        """Costmap origin based on robot center (rolling window)"""
        if self.rolling_window:
            half_w = self.map_width * self.resolution * 0.5
            half_h = self.map_height * self.resolution * 0.5
            self.origin_x = self.robot_x - half_w
            self.origin_y = self.robot_y - half_h
        else:
            self.origin_x = 0.0
            self.origin_y = 0.0

        self.costmap.info.origin.position.x = self.origin_x
        self.costmap.info.origin.position.y = self.origin_y
        self.costmap.info.origin.position.z = 0.0
        self.costmap.info.origin.orientation = Quaternion(w=1.0)

    def imu_callback(self, msg: Imu):
        self.current_imu = msg
        self.get_logger().debug("IMU data received.")

    def odom_callback(self, msg: Odometry):
        self.current_odom = msg

        # 로봇의 odom 프레임 위치
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        odom_z = msg.pose.pose.position.z

        # 오리엔테이션 추출
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        odom_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Transform odom frame to base_link frame
        try:
            # 최신 변환을 사용하도록 Time()을 사용 (즉, 현재 시점)
            transform_stamped = self.tf_buffer.lookup_transform(
                self.base_frame,        # target frame: base_link
                "odom",                 # source frame: odom
                rclpy.time.Time()       # latest available transform
            )

            # 변환을 PyKDL Frame으로 변환
            frame_kdl = self.transform_to_kdl(transform_stamped.transform)

            # odom 위치를 base_link로 변환
            in_vec = PyKDL.Vector(odom_x, odom_y, odom_z)
            out_vec = frame_kdl * in_vec

            # 변환된 위치
            self.robot_x = out_vec[0]
            self.robot_y = out_vec[1]

            # 로봇의 yaw는 base_link 프레임에서의 yaw를 사용
            # 정확한 yaw 변환을 위해 quaternion 변환 사용
            q_base = transform_stamped.transform.rotation
            roll, pitch, yaw_transform = euler_from_quaternion((q_base.x, q_base.y, q_base.z, q_base.w))
            transformed_yaw = odom_yaw + yaw_transform
            self.robot_yaw = transformed_yaw

            # 변환된 로봇 위치와 속도를 Odometry 메시지로 발행
            transformed_odom = Odometry()
            transformed_odom.header = transform_stamped.header
            transformed_odom.pose.pose.position.x = self.robot_x
            transformed_odom.pose.pose.position.y = self.robot_y
            transformed_odom.pose.pose.position.z = 0.0
            transformed_odom.pose.pose.orientation = transform_stamped.transform.rotation

            # Velocities 변환 (linear and angular)
            # Assume linear velocities are in odom frame; rotate to base_link frame
            linear_vel = msg.twist.twist.linear
            angular_vel = msg.twist.twist.angular

            # Transform linear velocity
            linear_kdl = PyKDL.Vector(linear_vel.x, linear_vel.y, linear_vel.z)
            transformed_linear = frame_kdl.M * linear_kdl
            transformed_odom.twist.twist.linear.x = transformed_linear.x()
            transformed_odom.twist.twist.linear.y = transformed_linear.y()
            transformed_odom.twist.twist.linear.z = transformed_linear.z()

            # Transform angular velocity
            angular_kdl = PyKDL.Vector(angular_vel.x, angular_vel.y, angular_vel.z)
            transformed_angular = frame_kdl.M * angular_kdl
            transformed_odom.twist.twist.angular.x = transformed_angular.x()
            transformed_odom.twist.twist.angular.y = transformed_angular.y()
            transformed_odom.twist.twist.angular.z = transformed_angular.z()

            # Publish transformed odometry
            self.transformed_odometry_pub.publish(transformed_odom)

            self.get_logger().info(f"Transformed Odometry: x={self.robot_x:.2f}, y={self.robot_y:.2f}, yaw={self.robot_yaw:.2f}")

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Failed to transform odom->base_link: {e}")

    def goal_callback(self, msg: PoseStamped):
        self.current_goal = msg
        self.arrived = False
        self.get_logger().info(
            f"New goal received: (x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}). "
            f"Reset arrived=False"
        )

    # -------------------- transform_to_kdl --------------------
    def transform_to_kdl(self, transform):
        """
        transform: geometry_msgs/Transform
          (NOT TransformStamped)
        """
        rot = PyKDL.Rotation.Quaternion(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w
        )
        trans = PyKDL.Vector(
            transform.translation.x,
            transform.translation.y,
            transform.translation.z
        )
        return PyKDL.Frame(rot, trans)

    def do_transform_cloud_kdl(self, cloud_in: PointCloud2, transform_stamped):
        """
        Use transform_to_kdl(transform_stamped.transform),
        then apply it to each point in the cloud.
        """
        # transform_stamped.transform : geometry_msgs/Transform
        frame_kdl = self.transform_to_kdl(transform_stamped.transform)

        transformed_points = []
        for p_in in read_points(cloud_in, field_names=("x", "y", "z"), skip_nans=True):
            in_vec = PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            out_vec = frame_kdl * in_vec
            transformed_points.append((out_vec[0], out_vec[1], out_vec[2]))
        return transformed_points

    def lidar_callback(self, msg: PointCloud2):
        """
        LiDAR data transformation (velodyne -> base_frame) and costmap update
        """
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.base_frame,        # target frame
                msg.header.frame_id,    # source frame
                rclpy.time.Time()       # latest available
            )

            # Convert pointcloud
            new_points = self.do_transform_cloud_kdl(msg, transform_stamped)
            self.lidar_points = new_points

            # Debug: publish transformed points
            header = msg.header
            header.frame_id = self.base_frame
            transformed_cloud_msg = create_cloud_xyz32(header, self.lidar_points)
            self.transformed_cloud_pub.publish(transformed_cloud_msg)

            self.update_costmap_from_points()

            self.get_logger().debug(f"Updated costmap with {len(self.lidar_points)} points.")

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Failed to transform LiDAR points: {e}")

    def update_costmap_from_points(self):
        data = [0] * (self.map_width * self.map_height)
        obstacle_count = 0

        min_z = 0.0
        max_z = 1.5
        close_threshold = 0.05

        for (px, py, pz) in self.lidar_points:
            dist_robot = math.hypot(px, py)
            # If point is extremely close (<0.05m) or in z range => obstacle
            if (dist_robot < close_threshold) or (min_z <= pz <= max_z):
                mx = int((px - self.origin_x) / self.resolution)
                my = int((py - self.origin_y) / self.resolution)
                if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                    idx = my * self.map_width + mx
                    data[idx] = 100
                    obstacle_count += 1

        # Inflation
        inflate_cells = int(self.inflation_radius / self.resolution)
        inflated_data = data[:]
        for y in range(self.map_height):
            for x in range(self.map_width):
                idx = y * self.map_width + x
                if data[idx] == 100:
                    for dy in range(-inflate_cells, inflate_cells + 1):
                        for dx2 in range(-inflate_cells, inflate_cells + 1):
                            nx = x + dx2
                            ny = y + dy
                            if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                                dist = math.hypot(dx2, dy) * self.resolution
                                if dist <= self.inflation_radius:
                                    n_idx = ny * self.map_width + nx
                                    inflated_data[n_idx] = 100

        self.costmap.data = inflated_data
        self.get_logger().debug(f"Updated costmap: obstacle={obstacle_count}, inflated={inflated_data.count(100)}")

    def publish_costmap(self):
        self.costmap.header.stamp = self.get_clock().now().to_msg()
        self.costmap_pub.publish(self.costmap)

        if self.current_goal and self.current_odom:
            gx = self.current_goal.pose.position.x
            gy = self.current_goal.pose.position.y
            dist = math.hypot(gx - self.robot_x, gy - self.robot_y)

            arrived_msg = Bool(data=(dist < self.arrival_threshold))
            self.arrived_pub.publish(arrived_msg)

            self.get_logger().info(f"Published costmap. Obstacle count={self.costmap.data.count(100)}")
            self.get_logger().info(f"Robot position: x={self.robot_x:.2f}, y={self.robot_y:.2f}, yaw={self.robot_yaw:.2f}")
            self.get_logger().info(f"Goal position: x={gx:.2f}, y={gy:.2f}, distance={dist:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = CostmapGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
