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
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
import PyKDL
import tf2_ros

class CostmapGenerator(Node):
    def __init__(self):
        super().__init__('costmap_generator')

        # ----- Parameters -----
        self.declare_parameter('map_width', 100)
        self.declare_parameter('map_height', 100)
        self.declare_parameter('resolution', 0.05)

        self.declare_parameter('lidar_topic', '/velodyne_points')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odom/ground_truth')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('update_frequency', 10.0)
        self.declare_parameter('inflation_radius', 0.05)
        self.declare_parameter('rolling_window', True)
        self.declare_parameter('arrival_threshold', 0.3)

        # ----- Get Parameters -----
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

        # ----- TF Buffer & Listener -----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ----- OccupancyGrid Initialization -----
        self.costmap = OccupancyGrid()
        self.costmap.header.frame_id = self.base_frame
        self.costmap.info.resolution = self.resolution
        self.costmap.info.width = self.map_width
        self.costmap.info.height = self.map_height

        # Initial origin (fixed if rolling_window=False)
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.reset_costmap_origin()

        # Initial data (all free=0)
        self.costmap.data = [0] * (self.map_width * self.map_height)

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
        self.transformed_cloud_pub = self.create_publisher(
            PointCloud2, '/transformed_cloud', 10)

        # Internal state
        self.current_goal = None
        self.current_imu = None
        self.current_odom = None
        self.lidar_points = []
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Periodically publish costmap
        self.timer = self.create_timer(
            1.0 / self.update_frequency, self.publish_costmap)

    def reset_costmap_origin(self):
        """Set costmap.info.origin"""
        self.costmap.info.origin.position.x = self.origin_x
        self.costmap.info.origin.position.y = self.origin_y
        self.costmap.info.origin.position.z = 0.0
        self.costmap.info.origin.orientation = Quaternion(w=1.0)
        # self.get_logger().info(f"Costmap origin updated: x={self.origin_x}, y={self.origin_y}")


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
        frame_kdl = self.transform_to_kdl(transform_stamped.transform)

        transformed_points = []
        for p_in in point_cloud2.read_points(cloud_in, field_names=("x", "y", "z"), skip_nans=True):
            in_vec = PyKDL.Vector(p_in[0], p_in[1], p_in[2])
            out_vec = frame_kdl * in_vec
            transformed_points.append((out_vec[0], out_vec[1], out_vec[2]))
        return transformed_points

    # -------------------- Callback Functions --------------------
    def imu_callback(self, msg: Imu):
        # pitch/roll can be used
        self.current_imu = msg

    def odom_callback(self, msg: Odometry):
        self.current_odom = msg
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Update origin if rolling window is enabled
        if self.rolling_window:
            self.origin_x = - (self.map_width * self.resolution) / 2.0
            self.origin_y = - (self.map_height * self.resolution) / 2.0
            self.reset_costmap_origin()

        self.get_logger().info(f"Robot position updated: x={self.robot_x}, y={self.robot_y}")
        # self.get_logger().info(f"Costmap origin: x={self.origin_x}, y={self.origin_y}")

    def goal_callback(self, msg: PoseStamped):
        self.current_goal = msg

    def lidar_callback(self, msg: PointCloud2):
        """
        LiDAR data transformation (velodyne -> base_frame) and costmap update.
        """
        try:
            # Get the latest transform from velodyne (msg.header.frame_id) to base_link
            transform_stamped = self.tf_buffer.lookup_transform(
                self.base_frame,              # Target frame (base_link)
                msg.header.frame_id,          # Source frame (velodyne)
                rclpy.time.Time()             # Latest transform
            )

            # Transform point cloud to base_link frame
            self.lidar_points = self.do_transform_cloud_kdl(msg, transform_stamped)

            # Debug: Log the number of transformed points
            # self.get_logger().info(f"Transformed {len(self.lidar_points)} LiDAR points to {self.base_frame} frame.")

            # Publish transformed point cloud for visualization (Optional)
            transformed_cloud_msg = create_cloud_xyz32(
                header=msg.header,
                points=self.lidar_points
            )
            transformed_cloud_msg.header.frame_id = self.base_frame
            self.transformed_cloud_pub.publish(transformed_cloud_msg)

            # Update the costmap with transformed points
            self.update_costmap_from_points()

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Failed to transform LiDAR points: {e}")

    # -------------------- Costmap Update Logic --------------------
    def update_costmap_from_points(self):
        """
        Obstacle detection and costmap update with inflation.
        """
        data = [0] * (self.map_width * self.map_height)

        min_z = 0.0
        max_z = 1.5
        close_threshold = 0.01  # 5cm proximity

        for (px, py, pz) in self.lidar_points:
            # Distance from robot
            dist_robot = math.hypot(px, py)
            if dist_robot < close_threshold:
                obstacle = True
            else:
                obstacle = min_z < pz < max_z

            if obstacle:
                # Debug: log detected obstacle
                self.get_logger().debug(f"Obstacle detected at: ({px}, {py}, {pz})")

                # base_link coords -> costmap index
                mx = int((px - self.costmap.info.origin.position.x) / self.resolution)
                my = int((py - self.costmap.info.origin.position.y) / self.resolution)

                if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                    idx = my * self.map_width + mx
                    data[idx] = 100
                    self.get_logger().debug(f"Obstacle cell updated at grid: ({mx}, {my}), index: {idx}")

        # Inflation
        inflate_cells = int(self.inflation_radius / self.resolution)
        inflated_data = data[:]
        for y in range(self.map_height):
            for x in range(self.map_width):
                idx = y * self.map_width + x
                if data[idx] == 100:
                    for dy in range(-inflate_cells, inflate_cells + 1):
                        for dx in range(-inflate_cells, inflate_cells + 1):
                            nx = x + dx
                            ny = y + dy
                            if 0 <= nx < self.map_width and 0 <= ny < self.map_height:
                                dist = math.sqrt(dx * dx + dy * dy) * self.resolution
                                if dist <= self.inflation_radius:
                                    n_idx = ny * self.map_width + nx
                                    inflated_data[n_idx] = 100
                                    self.get_logger().debug(f"Inflated cell: ({nx}, {ny}), index: {n_idx}")

        self.costmap.data = inflated_data


    def publish_costmap(self):
        # Update header stamp
        self.costmap.header.stamp = self.get_clock().now().to_msg()
        self.costmap_pub.publish(self.costmap)

        # Goal arrival check
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
