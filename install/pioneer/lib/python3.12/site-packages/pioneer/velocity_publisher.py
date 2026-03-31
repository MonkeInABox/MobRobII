import csv
import math
import os
import random

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, NavSatFix

NAVIGATING = 'navigating'
WALL_FOLLOWING = 'wall_following'
ROTATING = 'rotating'


class VelocityPublisher(Node):
    def __init__(self):
        super().__init__("velocity_publisher")

        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscription = self.create_subscription(
            LaserScan, "scan", self.scan_callback, 10
        )
        self.odom_subscription = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )
        self.gps_subscription = self.create_subscription(
            NavSatFix, "navsat/fix", self.gps_callback, 10
        )

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # imu bias
        self.bias = 0.005
        #noise 
        self.noise_std = 0.005

        self.gps_lat = None
        self.gps_lon = None
        self.gps_received = False
        self.origin_lat = -31.9080057
        self.origin_lon = 115.817984

        self.waypoints = []
        self.current_wp_ind = 0
        self.goal_reached_range = 0.3
        self.ang_speed_prop = 1.5
        self.linear_speed = 0.5
        self.yaw_tolerance = 0.05 
        self.i = 0

        self.ranges = []
        self.obstacle_detected = False
        self.stop_distance = 0.8
        self.wall_distance = 0.5

        self.state = NAVIGATING
        self.hit_point_x = 0.0
        self.hit_point_y = 0.0
        self.min_dist_to_goal = float('inf')

        csv_path = os.path.join(
            get_package_share_directory('pioneer'), 'waypoints.csv'
        )
        self.load_waypoints(csv_path)

    def load_waypoints(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f'Waypoints not found: {path}')
            return

        with open(path, newline='') as f:
            reader = csv.reader(f)
            for row in reader:
                if len(row) < 2:
                    continue
                try:
                    lat = float(row[0])
                    lon = float(row[1])
                    y = (lat - self.origin_lat) * 110000
                    x = (lon - self.origin_lon) * 85000
                    #https://www.movable-type.co.uk/scripts/latlong.html
                    # R = 6371000.0
                    # dlat = math.radians(lat - self.origin_lat)
                    # dlon = math.radians(lon - self.origin_lon)
                    # x = R * dlon * math.cos(math.radians(self.origin_lat))
                    # y = R * dlat *math.sin(math.radians(self.origin_lon))
                    target_yaw = None
                    if len(row) >= 3 and row[2].strip() != '':
                        target_yaw = math.radians(float(row[2].strip()))

                    self.waypoints.append((x, y, target_yaw))
                except ValueError:
                    continue

        self.get_logger().info(f'Waypoints: {self.waypoints}')
    
    def apply_imu_slip(self, true_yaw):
        #add some slip to thje imu for the simulation 
        # TODO remove if used on real robot
        noise = random.gauss(0.0, self.noise_std)
        slipped = true_yaw + noise + self.bias
        return math.atan2(math.sin(slipped), math.cos(slipped))

    def dist_to_goal(self):
        if self.current_wp_ind >= len(self.waypoints):
            return 0.0
        target_x, target_y, _ = self.waypoints[self.current_wp_ind]
        return math.sqrt((target_x - self.x) ** 2 + (target_y - self.y) ** 2)

    def gps_callback(self, msg):
        self.gps_lat = msg.latitude
        self.gps_lon = msg.longitude
        self.gps_received = True
        x, y = self.gps_to_xy(msg.latitude, msg.longitude)
        self.get_logger().info(
            f'GPS: lat={msg.latitude:.7f} lon={msg.longitude:.7f} \n'
            f'local x={x:.2f} y={y:.2f}'
        )

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        true_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.yaw = self.apply_imu_slip(true_yaw)

    def scan_callback(self, msg):
        self.ranges = list(msg.ranges)
        valid_ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if not valid_ranges:
            return
        for range in self.ranges:
            if range > 100:
                range = 100
        min_distance = min(valid_ranges)
        if min_distance < self.stop_distance:
            self.obstacle_detected = True

    def get_sector_min(self, ranges, angle_min_deg, angle_max_deg):
        if not ranges:
            return float('inf')
        n = len(ranges)
        i_min = int((angle_min_deg + 180) / 360 * n)
        i_max = int((angle_max_deg + 180) / 360 * n)
        i_min = max(0, min(i_min, n - 1))
        i_max = max(0, min(i_max, n - 1))
        sector = ranges[i_min:i_max] if i_min < i_max else ranges[i_max:i_min]
        valid = [r for r in sector if not math.isinf(r) and not math.isnan(r) and r > 0]
        return min(valid) if valid else 1000.0

    def rotate_to_yaw(self, target_yaw):
        """Rotate in place toward target_yaw. Returns True when complete."""
        msg = Twist()
        yaw_error = target_yaw - self.yaw
        # nomarlise
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        if abs(yaw_error) < self.yaw_tolerance:
            # Stop rotation
            self.publisher_.publish(msg)
            return True

        msg.linear.x = 0.0
        msg.angular.z = self.ang_speed_prop * yaw_error
        self.publisher_.publish(msg)
        self.get_logger().info(f'ROTATING | target: {math.degrees(target_yaw):.1f} | 'f'current: {math.degrees(self.yaw):.1f} | ')
        return False

    def timer_callback(self):
        msg = Twist()

        if self.current_wp_ind >= len(self.waypoints):
            return True

        target_x, target_y, target_yaw = self.waypoints[self.current_wp_ind]
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        if self.state == ROTATING:
            done = self.rotate_to_yaw(target_yaw)
            if done:
                self.get_logger().info(
                    f'Waypoint {self.current_wp_ind + 1} final yaw reached'
                )
                self.current_wp_ind += 1
                self.state = NAVIGATING
                self.min_dist_to_goal = float('inf')
            return

        if distance < self.goal_reached_range:
            self.get_logger().info(f'{self.current_wp_ind + 1} reached')
            if target_yaw is not None:
                self.state = ROTATING
            else:
                self.current_wp_ind += 1
                self.state = NAVIGATING
                self.min_dist_to_goal = float('inf')
            return

        if self.state == NAVIGATING:
            self.navigate_to_goal(dx, dy, distance)
        elif self.state == WALL_FOLLOWING:
            self.follow_wall()

    def navigate_to_goal(self, dx, dy, distance):
        msg = Twist()
        front = self.get_sector_min(self.ranges, -30, 30)

        if front < self.stop_distance:
            self.state = WALL_FOLLOWING
            self.hit_point_x = self.x
            self.hit_point_y = self.y
            self.min_dist_to_goal = self.dist_to_goal()
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
        self.get_logger().info(f'NAV | WP {self.current_wp_ind + 1} | \n'f'dist: {distance:.2f} | angle_err: {math.degrees(angle_error):.1f}')

    def follow_wall(self):
        msg = Twist()
        front = self.get_sector_min(self.ranges, -30, 30)
        right = self.get_sector_min(self.ranges, -90, -30)
        current_dist = self.dist_to_goal()

        dist_from_hit = math.sqrt(
            (self.x - self.hit_point_x) ** 2 +
            (self.y - self.hit_point_y) ** 2
        )
        #TODO angle to hit point and read that scan in for distbug
        if (dist_from_hit > 1.0 and
                current_dist < self.min_dist_to_goal and
                front > self.stop_distance):
            self.state = NAVIGATING
            return

        if current_dist < self.min_dist_to_goal:
            self.min_dist_to_goal = current_dist

        if front < self.stop_distance:
            msg.linear.x = 0.0
            msg.angular.z = self.linear_speed * 2.0
        elif right < self.wall_distance:
            msg.linear.x = self.linear_speed * 0.2
            msg.angular.z = self.linear_speed
        elif right > self.wall_distance:
            msg.linear.x = self.linear_speed * 0.2
            msg.angular.z = -self.linear_speed
        else:
            msg.linear.x = self.linear_speed
            msg.angular.z = 0.0

        self.publisher_.publish(msg)
        self.get_logger().info(f'WALL FOLLOWING | front: {front:.2f}m  \n'f'right: {right:.2f} | dist_to_goal: {current_dist:.2f}m')


def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    rclpy.spin(velocity_publisher)
    velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()