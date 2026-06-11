#!/usr/bin/env python3
"""
Simple waypoint follower: subscribes to /planned_path and /start_navigation,
tracks waypoints by publishing cmd_vel. Uses pure pursuit steering.
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.declare_parameter('lookahead_distance', 0.4)
        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.lookahead = self.get_parameter('lookahead_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.path = []
        self.current_waypoint_idx = 0
        self.navigating = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.has_odom = False

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.nav_status_pub = self.create_publisher(Bool, '/navigation_status', 10)

        self.create_subscription(Path, '/planned_path', self.path_cb, 10)
        self.create_subscription(Bool, '/start_navigation', self.start_nav_cb, 10)
        self.create_subscription(Bool, '/stop_navigation', self.stop_nav_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Twist, '/web_cmd_vel', self.web_cmd_vel_cb, 10)

        self.timer = self.create_timer(0.05, self.control_loop)
        self.manual_override = False
        self.manual_timeout = 0.0

        self.get_logger().info('Waypoint follower ready')

    def path_cb(self, msg: Path):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.current_waypoint_idx = 0
        self.get_logger().info(f'Received path with {len(self.path)} waypoints')

    def start_nav_cb(self, msg: Bool):
        if msg.data and len(self.path) > 0:
            self.navigating = True
            self.current_waypoint_idx = 0
            self.manual_override = False
            self.get_logger().info('Navigation started')
        elif not msg.data:
            self.get_logger().info('Navigation not executed (display only)')

    def stop_nav_cb(self, msg: Bool):
        if msg.data:
            self.navigating = False
            self.publish_stop()
            self.get_logger().info('Navigation stopped')

    def web_cmd_vel_cb(self, msg: Twist):
        has_cmd = (abs(msg.linear.x) > 0.01 or abs(msg.linear.y) > 0.01 or abs(msg.angular.z) > 0.01)
        if has_cmd and self.navigating:
            self.manual_override = True
            self.manual_timeout = self.get_clock().now().nanoseconds / 1e9 + 0.5
            self.cmd_pub.publish(msg)
        elif has_cmd:
            self.cmd_pub.publish(msg)

    def odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)
        self.has_odom = True

    def control_loop(self):
        if not self.navigating or not self.has_odom:
            return

        now_s = self.get_clock().now().nanoseconds / 1e9
        if self.manual_override:
            if now_s < self.manual_timeout:
                return
            self.manual_override = False

        if self.current_waypoint_idx >= len(self.path):
            self.navigating = False
            self.publish_stop()
            self.get_logger().info('Navigation complete - goal reached')
            status_msg = Bool()
            status_msg.data = False
            self.nav_status_pub.publish(status_msg)
            return

        # Find lookahead point
        target_idx = self.current_waypoint_idx
        for i in range(self.current_waypoint_idx, len(self.path)):
            dx = self.path[i][0] - self.robot_x
            dy = self.path[i][1] - self.robot_y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > self.lookahead:
                target_idx = i
                break
            target_idx = i

        # Advance waypoint index past reached waypoints
        while self.current_waypoint_idx < len(self.path) - 1:
            dx = self.path[self.current_waypoint_idx][0] - self.robot_x
            dy = self.path[self.current_waypoint_idx][1] - self.robot_y
            if math.sqrt(dx * dx + dy * dy) < self.goal_tolerance:
                self.current_waypoint_idx += 1
            else:
                break

        # Check if final goal reached
        final = self.path[-1]
        dx_goal = final[0] - self.robot_x
        dy_goal = final[1] - self.robot_y
        if math.sqrt(dx_goal * dx_goal + dy_goal * dy_goal) < self.goal_tolerance:
            self.navigating = False
            self.publish_stop()
            self.get_logger().info('Navigation complete - goal reached')
            return

        # Pure pursuit
        tx, ty = self.path[target_idx]
        dx = tx - self.robot_x
        dy = ty - self.robot_y
        angle_to_target = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_target - self.robot_yaw)

        cmd = Twist()
        if abs(angle_error) > 0.8:
            cmd.linear.x = 0.05
            cmd.angular.z = max(-self.max_angular, min(self.max_angular, angle_error * 2.0))
        else:
            cmd.linear.x = self.linear_speed * (1.0 - abs(angle_error) / 1.5)
            cmd.linear.x = max(0.05, cmd.linear.x)
            cmd.angular.z = max(-self.max_angular, min(self.max_angular, angle_error * 1.5))

        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
