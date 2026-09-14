#!/usr/bin/env python3
"""
测试路径发布器（配合 path_follower 验证路径跟踪功能）

功能：
    每隔 1 秒向 /planned_path 话题发布一条固定的测试路径。
    路径为一条包含两次转弯的折线，在 odom 坐标系中表达：
        (0,0) -> (0.5,0) -> (0.5,0.5) -> (1.0,0.5)
    每条线段被等分为 10 个点，共 33 个路径点。

    本节点仅用于测试 path_follower，不控制机器人运动。
    正式使用时可替换为 Nav2 规划器输出的路径。

参数（可用 --ros-args -p 覆盖）：
    frame_id     路径所在的坐标系，默认 'odom'
    path_topic   路径话题名，默认 '/planned_path'

用法：
    ros2 run neobot_control path_demo

注意：
    本节点不发布速度指令，可与 obstacle_stop + path_follower 同时运行。
    不要与 Nav2 同时使用（会抢占 /planned_path 话题）。
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class PathDemo(Node):
    """测试路径发布器：每秒发布一条固定折线路径到 /planned_path。"""

    def __init__(self):
        super().__init__('path_demo')

        # ---- 声明并读取参数 ----
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('path_topic', '/planned_path')
        self.frame_id = self.get_parameter('frame_id').value
        path_topic = self.get_parameter('path_topic').value

        # 路径发布器
        self.publisher = self.create_publisher(Path, path_topic, 10)
        # 每 1 秒发布一次路径（nav2 规划器通常也是周期性发布）
        self.create_timer(1.0, self._publish_path)
        self.get_logger().info(
            f'Publishing demo path on {path_topic} in frame {self.frame_id}'
        )

    def _publish_path(self):
        """构造并发布测试折线路径。"""
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        # 定义三段线段：((起点x,起点y), (终点x,终点y), 该段朝向yaw)
        # 第1段：沿 x 轴正方向行驶 0.5m
        # 第2段：左转90°，沿 y 轴正方向行驶 0.5m
        # 第3段：右转90°，沿 x 轴正方向行驶 0.5m
        segments = [
            ((0.0, 0.0), (0.5, 0.0), 0.0),                # 水平向前
            ((0.5, 0.0), (0.5, 0.5), math.pi / 2.0),     # 左转向上
            ((0.5, 0.5), (1.0, 0.5), 0.0),                # 右转向前
        ]
        # 每条线段等分为 10 个点，共 33 个点（3段 x 10点 + 1个终点）
        for start, end, yaw in segments:
            for step in range(11):
                ratio = step / 10.0
                pose = PoseStamped()
                pose.header = path.header
                # 线性插值计算每个路径点的坐标
                pose.pose.position.x = start[0] + ratio * (end[0] - start[0])
                pose.pose.position.y = start[1] + ratio * (end[1] - start[1])
                # 该段的朝向（四元数，仅 Z 轴旋转）
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                path.poses.append(pose)

        self.publisher.publish(path)


def main(args=None):
    """节点入口：初始化 rclpy，创建路径发布节点，保持运行。"""
    rclpy.init(args=args)
    node = PathDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
