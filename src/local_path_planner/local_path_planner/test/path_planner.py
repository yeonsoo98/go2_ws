#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Pose
from std_msgs.msg import Bool
from local_path_planner.algorithms.dwa import DWAAlgorithm
from local_path_planner.algorithms.teb import TEBAlgorithm
from tf_transformations import euler_from_quaternion
import math

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')

        # ---- Parameters ----
        self.declare_parameter('algorithm_name', 'dwa')
        self.declare_parameter('goal_arrival_tolerance', 1.0)
        self.declare_parameter('plan_loop_rate', 5.0)  # Hz

        self.algorithm_name = self.get_parameter('algorithm_name').get_parameter_value().string_value
        self.goal_arrival_tolerance = self.get_parameter('goal_arrival_tolerance').get_parameter_value().double_value
        self.plan_loop_rate = self.get_parameter('plan_loop_rate').get_parameter_value().double_value

        # ---- Select Algorithm ----
        if self.algorithm_name == 'dwa':
            self.planner = DWAAlgorithm()
            self.get_logger().info("PathPlanner using DWAAlgorithm")
        else:
            self.planner = TEBAlgorithm()
            self.get_logger().info("PathPlanner using TEBAlgorithm")

        # ---- Subscribers ----
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap', self.costmap_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/transformed_odometry', self.transformed_odometry_callback, 10)  # Updated Subscriber
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # ---- Publishers ----
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---- Internal State ----
        self.current_costmap = None
        self.current_odometry = None  # Holds the latest transformed odometry data
        self.current_goal = None
        self.arrived = False

        # ---- Timers ----
        plan_timer_period = 1.0 / self.plan_loop_rate  # For regular planning
        self.plan_timer = self.create_timer(plan_timer_period, self.update_cmd_vel)

        log_timer_period = 5.0  # Log every 5 seconds
        self.log_timer = self.create_timer(log_timer_period, self.log_state)

    def costmap_callback(self, msg: OccupancyGrid):
        self.current_costmap = msg
        obstacle_count = msg.data.count(100)
        self.get_logger().debug(f"Costmap received. Obstacle count: {obstacle_count}")

    def transformed_odometry_callback(self, msg: Odometry):
        """
        Callback for transformed odometry data in base_link frame.
        """
        self.current_odometry = msg

        # Extract robot position
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.robot_yaw = yaw

        # Extract velocities
        self.linear_velocity = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.angular_velocity = msg.twist.twist.angular.z

        self.get_logger().debug(
            f"Transformed Odometry - Position: ({self.robot_x:.2f}, {self.robot_y:.2f}), Yaw: {self.robot_yaw:.2f}, "
            f"Linear Vel: {self.linear_velocity:.2f}, Angular Vel: {self.angular_velocity:.2f}"
        )

    def goal_callback(self, msg: PoseStamped):
        self.current_goal = msg
        self.arrived = False
        self.get_logger().info(
            f"New goal received: (x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}). "
            f"Reset arrived=False"
        )

    def update_cmd_vel(self):
        """
        Regularly execute the planner
        """
        # 1) No goal -> stop
        if self.current_goal is None:
            self.publish_stop()
            self.get_logger().debug("No goal set. Publishing stop command.")
            return

        # 2) Costmap or transformed odometry not ready -> stop
        if self.current_costmap is None or self.current_odometry is None:
            self.publish_stop()
            self.get_logger().debug("Costmap or odometry data not ready. Publishing stop command.")
            return

        # 3) Already at goal -> stop
        if self.is_goal_reached():
            if not self.arrived:
                self.get_logger().info("Goal reached! Stop.")
            self.arrived = True
            self.publish_stop()
            return

        # 4) Execute planner
        cmd = self.planner.compute_velocity(
            costmap=self.current_costmap,
            odom=self.current_odometry,  # Pass transformed odometry
            goal=self.current_goal
        )

        self.get_logger().info(
            f"Publishing cmd_vel: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}"
        )

        self.cmd_vel_pub.publish(cmd)

    def log_state(self):
        """
        Log robot and goal state every 5 seconds
        """
        if self.current_odometry:
            rx = self.robot_x
            ry = self.robot_y
            ryaw = self.robot_yaw
            lv = self.linear_velocity
            av = self.angular_velocity
            self.get_logger().info(
                f"Robot position: x={rx:.2f}, y={ry:.2f}, yaw={ryaw:.2f}, linear_v={lv:.2f}, angular_v={av:.2f}"
            )
        if self.current_goal:
            gx = self.current_goal.pose.position.x
            gy = self.current_goal.pose.position.y
            self.get_logger().info(
                f"Goal position: x={gx:.2f}, y={gy:.2f}"
            )

    def is_goal_reached(self):
        # Simple check
        if self.current_goal is None or self.current_odometry is None:
            return False
        gx = self.current_goal.pose.position.x
        gy = self.current_goal.pose.position.y
        rx = self.robot_x
        ry = self.robot_y
        dist = math.sqrt((gx - rx)**2 + (gy - ry)**2)

        self.get_logger().debug(f"Goal distance: {dist:.2f} / tol={self.goal_arrival_tolerance:.2f}")
        return (dist < self.goal_arrival_tolerance)

    def publish_stop(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.get_logger().debug("Stop command => cmd_vel=0")
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
