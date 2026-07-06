#!/usr/bin/env python3
"""Persistent cmd_vel publisher for A1 robot exploration.
Maintains a single DDS connection to avoid instability with repeated
ros2 topic pub spawning.
"""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.segment_idx = 0
        self.segment_start = time.time()
        self.segments = self.build_path()
        self.timer = self.create_timer(0.1, self.publish_cmd)
        self.get_logger().info(f'Driver ready: {len(self.segments)} segments')

    def build_path(self):
        S = 0.3
        T = 0.3
        return [
            (S, 0.0, 33), (S, 0.1, 10), (S, 0.0, 20),
            (0.0, -T, 3), (S, 0.0, 15),
            (S, 0.0, 35),
            (0.0, T, 5), (S, 0.0, 20), (0.0, -T, 5), (S, 0.0, 20),
            (0.0, T, 10), (S, 0.0, 25),
            (0.0, 3.0, 2), (S, 0.0, 35),
            (0.0, T, 5), (S, 0.0, 25), (0.0, -T, 5), (S, 0.0, 25),
            (0.0, 3.0, 2), (S, 0.0, 35),
            (0.0, T, 5), (S, 0.0, 25), (0.0, -T, 5), (S, 0.0, 25),
        ]

    def publish_cmd(self):
        now = time.time()
        if self.segment_idx >= len(self.segments):
            self.pub.publish(Twist())
            return
        vx, wz, dur = self.segments[self.segment_idx]
        if now - self.segment_start >= dur:
            self.segment_idx += 1
            self.segment_start = now
            if self.segment_idx >= len(self.segments):
                self.get_logger().info('Exploration complete!')
                self.pub.publish(Twist())
                return
            vx, wz, dur = self.segments[self.segment_idx]
            self.get_logger().info(
                f'Seg {self.segment_idx}/{len(self.segments)}: vx={vx} wz={wz} {dur}s')
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = RobotDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
