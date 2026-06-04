"""
GNSS-Free Vineyard Navigation

Official implementation of the LiDAR/RANSAC navigation pipeline described in:

Betrò G., Pascuzzi S., Paciolla F.
"A GNSS-free LiDAR-based navigation architecture for autonomous inter-row
operation under sparse or absent vegetation conditions"
Smart Agricultural Technology, 2026.
DOI: 10.1016/j.atech.2026.102106

Copyright (C) 2026 Gerardo Betrò

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License v3.0.
"""

import random

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


class RowNavigationNode(Node):
    """LiDAR-based inter-row navigation node using RANSAC line fitting."""

    def __init__(self):
        super().__init__("row_navigation_node")

        # -------------------------
        # ROS parameters
        # -------------------------
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("marker_topic", "/line_markers")
        self.declare_parameter("frame_id", "base_link")

        self.declare_parameter("x_min", 0.5)
        self.declare_parameter("x_max", 18.0)
        self.declare_parameter("y_abs_max", 3.0)

        self.declare_parameter("linear_velocity", 0.5)
        self.declare_parameter("max_angular_velocity", 0.04)
        self.declare_parameter("k_yaw", 0.3)
        self.declare_parameter("k_lateral", 0.2)

        self.declare_parameter("ransac_iterations", 150)
        self.declare_parameter("ransac_threshold", 0.05)
        self.declare_parameter("ransac_sample_size", 3)
        self.declare_parameter("ransac_min_inliers", 4)
        self.declare_parameter("ransac_max_slope", 1.0)

        # -------------------------
        # Load parameters
        # -------------------------
        self.scan_topic = self.get_parameter("scan_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.marker_topic = self.get_parameter("marker_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.x_min = float(self.get_parameter("x_min").value)
        self.x_max = float(self.get_parameter("x_max").value)
        self.y_abs_max = float(self.get_parameter("y_abs_max").value)

        self.linear_velocity = float(self.get_parameter("linear_velocity").value)
        self.max_angular_velocity = float(
            self.get_parameter("max_angular_velocity").value
        )
        self.k_yaw = float(self.get_parameter("k_yaw").value)
        self.k_lateral = float(self.get_parameter("k_lateral").value)

        self.ransac_iterations = int(self.get_parameter("ransac_iterations").value)
        self.ransac_threshold = float(self.get_parameter("ransac_threshold").value)
        self.ransac_sample_size = int(self.get_parameter("ransac_sample_size").value)
        self.ransac_min_inliers = int(self.get_parameter("ransac_min_inliers").value)
        self.ransac_max_slope = float(self.get_parameter("ransac_max_slope").value)

        # -------------------------
        # ROS interfaces
        # -------------------------
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile,
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)

        self.get_logger().info("Row navigation node started.")

    # ------------------------------------------------------------------
    # RANSAC line fitting
    # ------------------------------------------------------------------
    def fit_line_ransac(self, x_data: np.ndarray, y_data: np.ndarray):
        """
        Fit a 2D line y = ax + b using RANSAC.

        Returns:
            a: line slope
            b: line intercept
            inlier_mask: boolean mask of inlier points
        """
        if len(x_data) < 2:
            return None, None, np.array([], dtype=bool)

        best_a = None
        best_b = None
        best_inlier_mask = np.zeros_like(x_data, dtype=bool)
        best_inlier_count = 0

        sample_size = max(2, min(self.ransac_sample_size, len(x_data)))

        for _ in range(self.ransac_iterations):
            sample_indices = random.sample(range(len(x_data)), sample_size)

            x_sample = x_data[sample_indices]
            y_sample = y_data[sample_indices]

            if np.allclose(x_sample, x_sample[0]):
                continue

            design_matrix = np.vstack(
                [x_sample, np.ones(len(x_sample))]
            ).T

            a, b = np.linalg.lstsq(
                design_matrix,
                y_sample,
                rcond=None,
            )[0]

            distances = np.abs(a * x_data - y_data + b) / np.sqrt(a**2 + 1.0)
            inlier_mask = distances < self.ransac_threshold
            inlier_count = int(np.sum(inlier_mask))

            if inlier_count > best_inlier_count:
                best_a = float(a)
                best_b = float(b)
                best_inlier_mask = inlier_mask
                best_inlier_count = inlier_count

        if best_a is None:
            return None, None, np.array([], dtype=bool)

        if best_inlier_count < self.ransac_min_inliers:
            return None, None, np.array([], dtype=bool)

        if abs(best_a) > self.ransac_max_slope:
            return None, None, np.array([], dtype=bool)

        return best_a, best_b, best_inlier_mask

    # ------------------------------------------------------------------
    # Main callback
    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        angles = np.linspace(
            msg.angle_min,
            msg.angle_max,
            len(msg.ranges),
        )

        ranges = np.asarray(msg.ranges, dtype=float)

        valid_mask = np.isfinite(ranges)
        ranges = ranges[valid_mask]
        angles = angles[valid_mask]

        x_all = ranges * np.cos(angles)
        y_all = ranges * np.sin(angles)

        roi_mask = (
            (x_all > self.x_min)
            & (x_all < self.x_max)
            & (np.abs(y_all) < self.y_abs_max)
        )

        x_roi = x_all[roi_mask]
        y_roi = y_all[roi_mask]

        left_mask = y_roi > 0.0
        right_mask = y_roi < 0.0

        x_left = x_roi[left_mask]
        y_left = y_roi[left_mask]

        x_right = x_roi[right_mask]
        y_right = y_roi[right_mask]

        a_left, b_left, inliers_left = self.fit_line_ransac(x_left, y_left)
        a_right, b_right, inliers_right = self.fit_line_ransac(x_right, y_right)

        if a_left is None or a_right is None:
            self.publish_stop_command()
            self.get_logger().warn("RANSAC failed: unable to estimate both row lines.")
            return

        # Center line estimation
        a_center = 0.5 * (a_left + a_right)
        b_center = 0.5 * (b_left + b_right)

        yaw_error = float(np.arctan(a_center))
        lateral_error = float(b_center)

        angular_velocity = self.k_yaw * yaw_error + self.k_lateral * lateral_error
        angular_velocity = float(
            np.clip(
                angular_velocity,
                -self.max_angular_velocity,
                self.max_angular_velocity,
            )
        )

        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = angular_velocity
        self.cmd_pub.publish(twist)

        # RViz visualization
        self.publish_line_marker(0, a_left, b_left, color=(0.0, 1.0, 0.0))
        self.publish_line_marker(1, a_right, b_right, color=(0.0, 0.0, 1.0))
        self.publish_line_marker(2, a_center, b_center, color=(0.0, 0.0, 0.0))

        self.publish_points_marker(
            10,
            x_left[inliers_left],
            y_left[inliers_left],
            color=(0.0, 1.0, 0.0),
        )

        self.publish_points_marker(
            11,
            x_right[inliers_right],
            y_right[inliers_right],
            color=(0.0, 0.0, 1.0),
        )

    # ------------------------------------------------------------------
    # Safety command
    # ------------------------------------------------------------------
    def publish_stop_command(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    # ------------------------------------------------------------------
    # RViz markers
    # ------------------------------------------------------------------
    def publish_line_marker(self, marker_id, a, b, color):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ransac_lines"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.05

        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0

        x_values = np.linspace(self.x_min, self.x_max, 30)
        y_values = a * x_values + b

        for x, y in zip(x_values, y_values):
            marker.points.append(
                Point(
                    x=float(x),
                    y=float(y),
                    z=0.0,
                )
            )

        self.marker_pub.publish(marker)

    def publish_points_marker(self, marker_id, x_data, y_data, color):
        if len(x_data) == 0:
            return

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ransac_inliers"
        marker.id = marker_id
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        marker.scale.x = 0.12
        marker.scale.y = 0.12

        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0

        for x, y in zip(x_data, y_data):
            marker.points.append(
                Point(
                    x=float(x),
                    y=float(y),
                    z=0.0,
                )
            )

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    node = RowNavigationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop_command()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
