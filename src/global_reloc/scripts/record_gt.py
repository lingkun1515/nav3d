#!/usr/bin/env python3
"""Live recorder: subscribe to super_lio /lio/odom and write a TUM trajectory
(timestamp tx ty tz qx qy qz qw) using the message header stamp (bag-time
domain, so it aligns with the data bag and with global_reloc query windows).

Run alongside `ros2 run super_lio relocation_node` + `ros2 bag play`:
  python3 record_gt.py --out runs/gt/gt.tum
Stop with Ctrl-C; it flushes on exit.

Note: rosbag2_py is unavailable in this container, so we record live (rclpy)
rather than post-processing a recorded bag.
"""
import argparse, signal, sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class GTRecorder(Node):
    def __init__(self, path, topic):
        super().__init__("gt_recorder")
        self.f = open(path, "w")
        self.n = 0
        self.sub = self.create_subscription(Odometry, topic, self.cb, 200)
        signal.signal(signal.SIGINT, self._flush)
        signal.signal(signal.SIGTERM, self._flush)

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.f.write(f"{t:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} "
                     f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n")
        self.n += 1
        if self.n % 200 == 0:
            self.f.flush()
            print(f"  gt poses: {self.n}", flush=True)

    def _flush(self, *a):
        try:
            self.f.flush(); self.f.close()
        except Exception:
            pass
        print(f"GT written: {self.n} poses", flush=True)
        sys.exit(0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--topic", default="/lio/odom")
    args = ap.parse_args()
    rclpy.init()
    node = GTRecorder(args.out, args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node._flush()


if __name__ == "__main__":
    main()
