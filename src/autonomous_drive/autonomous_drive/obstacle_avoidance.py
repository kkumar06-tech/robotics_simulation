#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Simple parameters (tweak these for your robot)
        self.max_speed = 0.5
        self.max_turn = 1.0
        self.safe_distance = 1.0
        self.danger_distance = 0.6

        # Cluster detection parameters
        self.cluster_angle_jump_deg = 3.0   # degrees between neighbors -> new cluster
        self.cluster_dist_jump = 0.30       # meters distance jump -> new cluster
        self.min_gap_width_deg = 5.0        # minimum gap width to be considered (degrees)
        self.max_point_distance = 10.0      # points farther than this treated as empty

        # Subscribe to PointCloud2
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.pc_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Store latest data (with smoothing)
        self.front_distance = 0.5
        self.left_distance = 0.5
        self.right_distance = 0.5
        self.first_scan_received = False

        # Gap detection output
        self.best_gap_angle = 0.0
        self.has_gap = False

        # State memory - prevents jerky movements
        self.turning_left = False
        self.turning_right = False
        self.turn_commit_timer = 0  # How many frames to keep turning

        self.get_logger().info("Cluster-based Obstacle Avoidance Started!")
        self.get_logger().info("Waiting for first sensor data...")

    def pc_callback(self, msg):
        """Extract only the relevant distance info from point cloud"""
        try:
            x_points = []
            y_points = []
            z_points = []

            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                x, y, z = point[0], point[1], point[2]
                # Only consider points at robot height
                if -0.5 < z < 1.5:
                    x_points.append(x)
                    y_points.append(y)
                    z_points.append(z)

            if not x_points:
                return

            x_points = np.array(x_points)
            y_points = np.array(y_points)

            # Calculate distances and angles
            distances = np.sqrt(x_points**2 + y_points**2)
            angles = np.arctan2(y_points, x_points)

            # Divide into three sectors: front, left, right
            front_mask = (angles > -np.pi/6) & (angles < np.pi/6)       # -30..+30 deg
            left_mask = (angles >= np.pi/6) & (angles < 5*np.pi/6)      # +30..+150 deg
            right_mask = (angles <= -np.pi/6) & (angles > -5*np.pi/6)   # -150..-30 deg

            # Get minimum distance in each sector (or a large value if none)
            front_points = distances[front_mask]
            left_points = distances[left_mask]
            right_points = distances[right_mask]

            # Temporal smoothing
            if not self.first_scan_received:
                smoothing = 1.0
                self.first_scan_received = True
                self.get_logger().info("First scan received!")
            else:
                smoothing = 0.3

            front_new = np.min(front_points) if len(front_points) > 0 else self.max_point_distance
            left_new = np.min(left_points) if len(left_points) > 0 else self.max_point_distance
            right_new = np.min(right_points) if len(right_points) > 0 else self.max_point_distance

            self.front_distance = (1 - smoothing) * self.front_distance + smoothing * front_new
            self.left_distance = (1 - smoothing) * self.left_distance + smoothing * left_new
            self.right_distance = (1 - smoothing) * self.right_distance + smoothing * right_new

            # Cluster-based gap detection
            self.find_best_direction(distances, angles)

        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")

    def find_best_direction(self, distances, angles):
        """Cluster-based gap detection tuned for RoboSense HeliOS-16 P LiDAR"""
        try:
            # Step 1 — Filter to front hemisphere (-120° to +120°)
            mask = (angles >= -2.09) & (angles <= 2.09)
            d = distances[mask]
            a = angles[mask]

            if len(d) == 0:
                self.has_gap = False
                return

            # Remove points beyond max distance
            far_mask = d <= self.max_point_distance
            if not np.any(far_mask):
                self.has_gap = False
                return
            d = d[far_mask]
            a = a[far_mask]

            # Step 2 — Sort points by angle
            order = np.argsort(a)
            d = d[order]
            a = a[order]

            # Step 3 — Build clusters
            clusters = []
            current_start = 0

            ANGLE_JUMP = np.radians(self.cluster_angle_jump_deg)
            DIST_JUMP = self.cluster_dist_jump

            for i in range(1, len(a)):
                angle_diff = abs(a[i] - a[i - 1])
                dist_diff = abs(d[i] - d[i - 1])

                if angle_diff > ANGLE_JUMP or dist_diff > DIST_JUMP:
                    cluster_size = i - current_start
                    if cluster_size >= 3:
                        clusters.append((current_start, i - 1))
                    current_start = i

            # Append last cluster
            cluster_size = len(a) - current_start
            if cluster_size >= 3:
                clusters.append((current_start, len(a) - 1))

            if not clusters or len(clusters) == 1:
                self.has_gap = False
                return

            # Step 4 — Find gaps between clusters
            gaps = []
            min_gap_width_rad = np.radians(self.min_gap_width_deg)

            for i in range(len(clusters) - 1):
                end_idx = clusters[i][1]
                start_next_idx = clusters[i + 1][0]

                left_angle = a[end_idx]
                right_angle = a[start_next_idx]

                gap_width = right_angle - left_angle

                if gap_width > min_gap_width_rad:
                    gaps.append((left_angle, right_angle, gap_width))

            if not gaps:
                self.has_gap = False
                return

            # Step 5 — Choose the largest gap
            largest_gap = max(gaps, key=lambda g: g[2])
            left_angle, right_angle, _ = largest_gap

            # Center of the gap
            self.best_gap_angle = (left_angle + right_angle) / 2.0
            self.has_gap = True

        except Exception as e:
            self.get_logger().error(f"Cluster Gap Detection Error: {e}")
            self.has_gap = False

    def control_loop(self):
        """Reactive control using gap detection"""
        if not self.first_scan_received:
            self.cmd_pub.publish(Twist())
            return

        cmd = Twist()

        gap_str = f" | Gap: {np.degrees(self.best_gap_angle):.0f}°" if self.has_gap else ""
        self.get_logger().info(
            f"Front: {self.front_distance:.2f}m | Left: {self.left_distance:.2f}m | Right: {self.right_distance:.2f}m{gap_str}",
            throttle_duration_sec=1.0
        )

        # Deadlock detection
        all_blocked = (self.front_distance < self.danger_distance and
                       self.left_distance < self.danger_distance and
                       self.right_distance < self.danger_distance)

        if all_blocked:
            cmd.linear.x = -0.3
            cmd.angular.z = self.max_turn
            self.get_logger().info("🔙 STUCK! Backing up and turning")
            self.cmd_pub.publish(cmd)
            return

        # Turn commit
        if self.turn_commit_timer > 0:
            self.turn_commit_timer -= 1
            if self.turning_left:
                cmd.angular.z = self.max_turn * 0.7
                if self.front_distance > self.safe_distance:
                    cmd.linear.x = self.max_speed * 0.5
            elif self.turning_right:
                cmd.angular.z = -self.max_turn * 0.7
                if self.front_distance > self.safe_distance:
                    cmd.linear.x = self.max_speed * 0.5
            self.cmd_pub.publish(cmd)
            return

        # Normal navigation
        if self.front_distance < self.danger_distance:
            cmd.linear.x = 0.0
            if self.has_gap and abs(self.best_gap_angle) < np.pi / 4:
                cmd.angular.z = float(np.clip(self.best_gap_angle * 2.5, -self.max_turn, self.max_turn))
                self.turning_left = cmd.angular.z > 0
                self.turning_right = cmd.angular.z < 0
                self.turn_commit_timer = 20
            else:
                if self.left_distance > self.right_distance:
                    cmd.angular.z = self.max_turn
                    self.turning_left = True
                    self.turning_right = False
                    self.turn_commit_timer = 20
                else:
                    cmd.angular.z = -self.max_turn
                    self.turning_right = True
                    self.turning_left = False
                    self.turn_commit_timer = 20
        elif self.front_distance < self.safe_distance:
            cmd.linear.x = self.max_speed * 0.3
            if not self.turning_left and not self.turning_right:
                if self.left_distance > self.right_distance * 1.2:
                    cmd.angular.z = self.max_turn * 0.5
                    self.turning_left = True
                    self.turn_commit_timer = 10
                else:
                    cmd.angular.z = -self.max_turn * 0.5
                    self.turning_right = True
                    self.turn_commit_timer = 10
            else:
                cmd.angular.z = self.max_turn * 0.5 if self.turning_left else -self.max_turn * 0.5
        else:
            cmd.linear.x = self.max_speed
            cmd.angular.z = 0.0
            self.turning_left = False
            self.turning_right = False

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.get_logger().info("Shutting down")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
