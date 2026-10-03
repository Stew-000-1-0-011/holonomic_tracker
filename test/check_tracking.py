#!/usr/bin/env python3
"""模擬ロボットの真値と目標を突き合わせて、追従誤差を表示する手動テスト。

    ros2 launch holonomic_tracker tracker_node.launch.py sim:=true
    ros2 run holonomic_tracker check_tracking.py   # 別端末で

目標は受信したものを真値の時刻まで速度FFで外挿して比べる。
"""

import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from holonomic_tracker.msg import TrackingReference
from rclpy.node import Node


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Checker(Node):
    def __init__(self, duration):
        super().__init__('check_tracking')
        self.ref = None
        self.samples = []
        self.duration = duration
        self.t0 = None
        self.create_subscription(TrackingReference, '/tracker_node/reference', self.on_ref, 10)
        self.create_subscription(PoseStamped, '/fake_holonomic_robot/truth', self.on_truth, 50)

    def on_ref(self, msg):
        self.ref = msg

    def on_truth(self, msg):
        if self.ref is None:
            return
        t = stamp_sec(msg.header.stamp)
        if self.t0 is None:
            self.t0 = t
        dt = t - stamp_sec(self.ref.header.stamp)
        rx = self.ref.x + self.ref.vx * dt
        ry = self.ref.y + self.ref.vy * dt
        ryaw = self.ref.yaw + self.ref.omega * dt
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # 立ち上がりの 3 s は捨てる
        if t - self.t0 > 3.0:
            self.samples.append((
                math.hypot(rx - msg.pose.position.x, ry - msg.pose.position.y),
                abs(wrap(ryaw - yaw)),
            ))
        if t - self.t0 > 3.0 + self.duration:
            self.report()
            raise SystemExit

    def report(self):
        n = len(self.samples)
        rms_p = math.sqrt(sum(p * p for p, _ in self.samples) / n)
        rms_y = math.sqrt(sum(y * y for _, y in self.samples) / n)
        max_p = max(p for p, _ in self.samples)
        print(f'samples {n}: rms pos {rms_p:.4f} m, max pos {max_p:.4f} m, rms yaw {rms_y:.4f} rad')


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    node = Checker(duration)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
