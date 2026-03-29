import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ament_index_python.packages import get_package_share_directory

import csv
import math
import os


class VelocityPublisher(Node):
    def __init__(self):
        super().__init__("velocity_publisher")
        #creating scanner
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscription = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            10
        )
        #create odom gps
        self.odom_subscription = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            10
        )

        timer_period = 0.1 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.obstacle_detected = False
        self.stop_distance = 0.5

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        #waypoints storage
        self.waypoints = []
        self.current_wp_ind = 0
        self.goal_reached_range = 0.3

        csv_path = os.path.join(
            get_package_share_directory('pioneer'), 'waypoints.csv'
        )
        self.load_waypoints(csv_path)

        #speed stuffs
        self.ang_speed_prop = 1.5
        self.linear_speed = 0.5

        self.i = 0
    
    def load_waypoints(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f'Waypoints not found')
            return
        
        with open(path, newline = '') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) < 2:
                    continue
                try:
                    x = float(row[0])
                    y = float(row[1])
                    self.waypoints.append((x, y))
                except ValueError:
                    continue
        
        self.get_logger().info(f'wp: {self.waypoints}')

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        # get rid of infinity, nan and 0
        valid_ranges = [
            r for r in msg.ranges
            if msg.range_min < r < msg.range_max
        ]
        if not valid_ranges:
            return 
        
        min_distance = min(valid_ranges)

        if min_distance < self.stop_distance:
            if not self.obstacle_detected:
                self.get_logger().info(
                    f'Obstacle detected at {min_distance:.2f}m'
                )
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def timer_callback(self):
        msg = Twist()
        if not self.waypoints:
            self.get_logger().info('No waypoints loaded.', once=True)
            self.publisher_.publish(msg)
            return

        if self.current_wp_ind >= len(self.waypoints):
            self.get_logger().info('All waypoints reached!', once=True)
            self.publisher_.publish(msg)
            return

        #stop @ obst
        if self.obstacle_detected:
            self.publisher_.publish(msg)
            return

        target_x, target_y = self.waypoints[self.current_wp_ind]
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        #wp reach checl
        if distance < self.goal_reached_range:
            self.get_logger().info(
                f'Waypoint {self.current_wp_ind + 1}/'
                f'{len(self.waypoints)} reached: ({target_x}, {target_y})'
            )
            self.current_wp_ind += 1
            return

        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        if abs(angle_error) > 0.3:
            msg.linear.x = 0.0
        else:
            msg.linear.x = self.linear_speed

        msg.angular.z = self.ang_speed_prop * angle_error

        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Waypoint {self.current_wp_ind + 1}/{len(self.waypoints)} | '
            f'dist: {distance:.2f}m | angle_err: {math.degrees(angle_error):.1f}°'
        )


def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()