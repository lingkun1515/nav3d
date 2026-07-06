#!/usr/bin/env python3
"""Drive the A1 robot through hospital_two_floors_stripped for SLAM mapping.

Uses cmd_vel to follow a waypoint path covering both floors via the ramp
at (x=-1.71, y=16.37). Robot speed: 0.3 m/s, turn rate: 0.3 rad/s.
Monitors /odom for position feedback.
"""
import math
import sys
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.got_odom = False

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Extract yaw from quaternion
        self.yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.got_odom = True

    def send_cmd(self, vx, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.cmd_pub.publish(msg)

    def stop(self):
        self.send_cmd(0.0, 0.0)

    def get_yaw_diff(self, target_yaw):
        diff = target_yaw - self.yaw
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def drive_to(self, target_x, target_y, speed=0.3, pos_tol=0.5, yaw_tol=0.1):
        """Drive to a target position. Returns True if reached."""
        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < pos_tol:
            self.stop()
            return True

        target_yaw = math.atan2(dy, dx)
        yaw_diff = self.get_yaw_diff(target_yaw)

        # First turn to face target, then drive forward
        if abs(yaw_diff) > yaw_tol:
            wz = 0.3 if yaw_diff > 0 else -0.3
            self.send_cmd(0.0, wz)
        else:
            # Drive forward, with slight yaw correction
            wz = max(-0.3, min(0.3, yaw_diff * 2.0))
            self.send_cmd(speed, wz)
        return False

    def turn_to(self, target_yaw, tol=0.1):
        """Turn to a specific yaw angle."""
        diff = self.get_yaw_diff(target_yaw)
        if abs(diff) < tol:
            self.stop()
            return True
        wz = 0.3 if diff > 0 else -0.3
        self.send_cmd(0.0, wz)
        return False


def main():
    rclpy.init()
    node = Explorer()

    # Waypoints covering ground floor corridors, then ramp, then second floor
    # Hospital layout: x in [-13, 13], y in [-32, 16], ramp at (-1.71, 16.37)
    # Ground floor waypoints (z=0)
    waypoints = [
        # Start area, drive forward along y axis
        (0.0, 5.0),
        (0.0, 10.0),
        # Move toward ramp
        (-1.0, 13.0),
        (-1.71, 16.37),   # Base of ramp
        # Up the ramp (drive forward, robot will ascend)
        (-1.71, 18.0),
        (-1.71, 20.0),
        # Second floor (z=3), explore
        (0.0, 15.0),
        (3.0, 12.0),
        (5.0, 8.0),
        (5.0, 0.0),
        (5.0, -5.0),
        (0.0, -10.0),
        (-3.0, -15.0),
        (-5.0, -20.0),
        (-5.0, -25.0),
        (0.0, -25.0),
        (5.0, -20.0),
        (8.0, -10.0),
        (8.0, 0.0),
        (8.0, 8.0),
        # Come back toward center
        (3.0, 10.0),
        (0.0, 10.0),
        (0.0, 5.0),
        (0.0, 0.0),
    ]

    current_wp = 0
    rate = node.create_rate(20)  # 20 Hz

    # Wait for odom
    node.get_logger().info("Waiting for odom...")
    while not node.got_odom and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info(f"Got odom: ({node.x:.2f}, {node.y:.2f}, yaw={math.degrees(node.yaw):.1f})")

    node.get_logger().info(f"Starting exploration with {len(waypoints)} waypoints")

    while rclpy.ok() and current_wp < len(waypoints):
        rclpy.spin_once(node, timeout_sec=0.01)
        wp = waypoints[current_wp]
        reached = node.drive_to(wp[0], wp[1])

        if reached:
            node.get_logger().info(f"Reached waypoint {current_wp}: ({wp[0]:.1f}, {wp[1]:.1f}) "
                                   f"actual: ({node.x:.2f}, {node.y:.2f})")
            current_wp += 1
            node.stop()
            time.sleep(0.5)  # brief pause at each waypoint
        else:
            if current_wp % 5 == 0:
                node.get_logger().info(f"WP {current_wp} ({wp[0]:.1f}, {wp[1]:.1f}) "
                                       f"pos: ({node.x:.2f}, {node.y:.2f}) yaw: {math.degrees(node.yaw):.1f}")

        rate.sleep()

    node.stop()
    node.get_logger().info("Exploration complete!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
