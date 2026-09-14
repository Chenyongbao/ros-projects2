#!/usr/bin/env python3
"""
自定义路径跟踪控制器（学习阶段 6：Pure Pursuit 路径跟踪）

功能：
    订阅 /planned_path（nav_msgs/Path）获取一系列航路点，
    结合 /odom 的里程计反馈，使用 Pure Pursuit（纯追踪）算法
    计算前视点并发布速度指令到 /cmd_vel_raw。

    Pure Pursuit 核心思路：
        1. 在路径上找到距离机器人最近的点；
        2. 从该点沿路径向前找到一个距离 >= lookahead_distance 的前视点；
        3. 将前视点转换到机器人本体坐标系，计算曲率 curvature = 2 * local_y / dist^2；
        4. 线速度 = speed，角速度 = speed * curvature；
        5. 接近终点时线性减速。

    输出 /cmd_vel_raw 经过 obstacle_stop 安全网关后到达 Gazebo，
    形成完整的控制链路：
        /planned_path + /odom --> path_follower --> /cmd_vel_raw
            --> obstacle_stop --> /cmd_vel --> Gazebo

参数（可用 --ros-args -p 覆盖）：
    lookahead_distance  前视距离，默认 0.25 m
    linear_speed        最大线速度，默认 0.2 m/s
    max_angular_speed   最大角速度，默认 0.8 rad/s
    goal_tolerance      终点到达容差，默认 0.08 m
    path_timeout        路径超时时间，默认 1.5 s（超过此时间未更新则停止）

用法：
    ros2 run neobot_control path_follower

注意：
    本节点输出到 /cmd_vel_raw（不是 /cmd_vel），必须配合 obstacle_stop 使用。
    运行前请停止 Nav2、teleop、square_driver、point_controller 等。
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


def normalize_angle(angle):
    """把任意角度归一化到 (-pi, pi] 区间，避免角度差跨 ±pi 出错。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lower, upper):
    """把数值夹在 [lower, upper] 区间内。"""
    return max(lower, min(value, upper))


class PathFollower(Node):
    """基于 Pure Pursuit 的路径跟踪控制器，订阅路径和里程计，输出速度指令。"""

    def __init__(self):
        super().__init__('path_follower')

        # ---- 声明并读取参数 ----
        self.declare_parameter('lookahead_distance', 0.25)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('goal_tolerance', 0.08)
        self.declare_parameter('path_timeout', 1.5)

        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.path_timeout = self.get_parameter('path_timeout').value

        # 速度指令输出到 /cmd_vel_raw，由 obstacle_stop 安全网关转发到 /cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        # 订阅里程计反馈，获取机器人当前位置
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        # 订阅路径话题，接收 nav_msgs/Path 格式的航路点序列
        self.create_subscription(
            Path,
            '/planned_path',
            self._path_callback,
            10,
        )
        # 50 Hz 控制周期
        self.create_timer(0.05, self._control_loop)

        # ---- 运行时状态 ----
        self.x = None       # 机器人当前 x 坐标（odom 坐标系）
        self.y = None       # 机器人当前 y 坐标
        self.yaw = None     # 机器人当前偏航角（弧度）
        self.path = []      # 当前路径点列表 [(x1,y1), (x2,y2), ...]
        self.path_stamp_ns = 0  # 最后一次收到路径的时间戳（纳秒）
        self.finished = False   # 是否已完成路径跟踪
        self._last_log_ns = 0   # 上一次打印进度日志的时间戳

        self.get_logger().info(
            'Waiting for /odom and /planned_path; output is /cmd_vel_raw'
        )

    def _odom_callback(self, message):
        """从里程计消息中提取机器人当前位姿（位置 + 偏航角）。"""
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.x = position.x
        self.y = position.y
        # 四元数反解绕 Z 轴的偏航角
        self.yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y
                          + orientation.z * orientation.z),
        )

    def _path_callback(self, message):
        """接收新路径：将 nav_msgs/Path 转为 [(x,y)] 列表，相同路径只刷新时间戳。"""
        if not message.poses:
            return
        # 提取所有路径点的 (x, y) 坐标
        new_path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in message.poses
        ]
        # 如果路径内容没变（只是定时重发），只刷新时间戳不重置 finished
        if new_path == self.path:
            self.path_stamp_ns = self.get_clock().now().nanoseconds
            return
        self.path = new_path
        self.path_stamp_ns = self.get_clock().now().nanoseconds
        self.finished = False
        self.get_logger().info(f'Received new path with {len(self.path)} poses')

    def _control_loop(self):
        """
        50 Hz 控制主循环：Pure Pursuit 算法实现。

        步骤：
            1. 检查路径是否新鲜、是否已到达终点；
            2. 在路径上找最近点，再从最近点向前找前视点；
            3. 将前视点转换到机器人本体坐标系；
            4. 计算曲率，输出线速度和角速度。
        """
        command = Twist()
        now_ns = self.get_clock().now().nanoseconds

        # 检查路径是否在超时时间内更新过
        path_is_fresh = (
            bool(self.path)
            and now_ns - self.path_stamp_ns <= self.path_timeout * 1e9
        )

        # 无里程计、路径过期或已完成：发布零速
        if self.x is None or not path_is_fresh or self.finished:
            self.cmd_vel_publisher.publish(command)
            return

        # 检查是否已到达路径终点（最后一个路径点）
        goal_x, goal_y = self.path[-1]
        goal_distance = math.hypot(goal_x - self.x, goal_y - self.y)
        if goal_distance <= self.goal_tolerance:
            self.finished = True
            self.get_logger().info(
                f'Path completed at x={self.x:.3f}, y={self.y:.3f}'
            )
            self.cmd_vel_publisher.publish(command)
            return

        # ---- 步骤 1：在路径上找距离机器人最近的点 ----
        closest_index = min(
            range(len(self.path)),
            key=lambda index: math.hypot(
                self.path[index][0] - self.x,
                self.path[index][1] - self.y,
            ),
        )

        # ---- 步骤 2：从最近点向前找前视点（距离 >= lookahead_distance） ----
        lookahead_index = len(self.path) - 1
        for index in range(closest_index, len(self.path)):
            distance = math.hypot(
                self.path[index][0] - self.x,
                self.path[index][1] - self.y,
            )
            if distance >= self.lookahead_distance:
                lookahead_index = index
                break

        # ---- 步骤 3：将前视点转换到机器人本体坐标系 ----
        target_x, target_y = self.path[lookahead_index]
        dx = target_x - self.x
        dy = target_y - self.y
        # 旋转矩阵：从 odom 坐标系变换到机器人本体坐标系
        local_x = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy
        local_y = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy

        # ---- 步骤 4：计算曲率并输出速度指令 ----
        # Pure Pursuit 曲率公式：curvature = 2 * local_y / dist^2
        lookahead_squared = max(dx * dx + dy * dy, 1e-6)
        curvature = 2.0 * local_y / lookahead_squared

        # 线速度：接近终点时线性减速（0.3m 以内开始减速）
        command.linear.x = clamp(
            self.linear_speed * min(1.0, goal_distance / 0.3),
            0.0,
            self.linear_speed,
        )
        # 如果前视点在机器人后方（local_x <= 0），不前进只转向
        if local_x <= 0.0:
            command.linear.x = 0.0
        # 角速度 = 线速度 * 曲率，夹在最大角速度范围内
        command.angular.z = clamp(
            command.linear.x * curvature,
            -self.max_angular_speed,
            self.max_angular_speed,
        )
        self.cmd_vel_publisher.publish(command)
        self._log_progress(goal_distance, lookahead_index)

    def _log_progress(self, goal_distance, lookahead_index):
        """每 2 秒打印一次跟踪进度，包括剩余距离、前视点索引和当前位姿。"""
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_log_ns < 2_000_000_000:
            return
        self._last_log_ns = now_ns
        self.get_logger().info(
            f'path progress: goal_distance={goal_distance:.3f}, '
            f'lookahead_index={lookahead_index}, '
            f'pose=({self.x:.3f}, {self.y:.3f}, {self.yaw:.3f})'
        )

    def stop(self):
        """退出时发布零速度，避免机器人继续运动。"""
        self.cmd_vel_publisher.publish(Twist())


def main(args=None):
    """节点入口：初始化 rclpy，创建路径跟踪节点，保持运行。"""
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
