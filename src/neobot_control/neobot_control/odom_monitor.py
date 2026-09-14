#!/usr/bin/env python3
"""
里程计监控节点（学习阶段 1：只读，不控制机器人）

功能：
    订阅 Gazebo 差速插件发布的 /odom 里程计话题，
    每 1 秒在终端打印一次机器人的位置 (x, y) 和偏航角 (yaw)。
    本节点不发布任何速度指令，可以与其他控制节点同时安全运行。

用法：
    ros2 run neobot_control odom_monitor
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class OdomMonitor(Node):
    """订阅 /odom 并按固定频率打印机器人位姿。"""

    def __init__(self):
        # 注册节点，节点名为 'odom_monitor'
        super().__init__('odom_monitor')
        # 记录上一次打印日志的时间戳（纳秒），用于限频
        self._last_log_ns = 0
        # 订阅里程计话题，队列深度 10（里程计为慢话题，小队列足够）
        self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10,
        )
        self.get_logger().info('Listening to /odom')

    def _odom_callback(self, message: Odometry):
        # ---- 日志限频：两次打印之间至少间隔 1 秒，避免刷屏 ----
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_log_ns < 1_000_000_000:
            return
        self._last_log_ns = now_ns

        # 取出位置（米）和姿态四元数
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        # 由四元数 (w, x, y, z) 反解出绕 Z 轴的偏航角（弧度）
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y
                          + orientation.z * orientation.z),
        )
        self.get_logger().info(
            f'pose: x={position.x:.3f}, y={position.y:.3f}, '
            f'yaw={yaw:.3f} rad'
        )


def main(args=None):
    # 初始化 rclpy 并创建节点
    rclpy.init(args=args)
    node = OdomMonitor()
    try:
        # 事件循环：保持节点存活，回调按消息驱动执行
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C 退出
        pass
    finally:
        # 释放节点资源并关闭 rclpy
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
