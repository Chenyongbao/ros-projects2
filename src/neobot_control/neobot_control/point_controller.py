#!/usr/bin/env python3
"""
定点到达控制器（学习阶段 3：基于里程计的点到点控制）

功能：
    让机器人行驶到 /odom 坐标系中指定的目标位姿 (x, y, yaw)。
    控制律：
        1. 距离未收敛：朝目标点方向行驶，角速度 = 1.8 * 航向误差；
           只有当航向误差小于 0.6 rad 时才前进（否则先原地转过来）；
        2. 距离已收敛：原地旋转对准目标朝向；
        3. 位置与朝向都在容差内：置完成标志，输出零速并提示到达。

参数（可用 --ros-args -p 覆盖）：
    target_x / target_y / target_yaw  目标位姿（odom 坐标系，默认 0.5 / 0.0 / 0.0）
    position_tolerance                位置容差，默认 0.05 m
    angle_tolerance                   角度容差，默认 0.05 rad
    max_linear_speed                  最大线速度，默认 0.25 m/s
    max_angular_speed                 最大角速度，默认 0.8 rad/s

用法：
    ros2 run neobot_control point_controller --ros-args \
      -p target_x:=0.5 -p target_y:=0.3 -p target_yaw:=1.57

注意：
    运行前请停止其他发布 /cmd_vel 的节点（teleop / Nav2 / square_driver 等）。
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def normalize_angle(angle):
    """把任意角度归一化到 (-pi, pi] 区间。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lower, upper):
    """把数值夹在 [lower, upper] 区间内。"""
    return max(lower, min(value, upper))


class PointController(Node):
    """基于里程计反馈的定点到达控制器。"""

    def __init__(self):
        super().__init__('point_controller')

        # ---- 声明并读取参数 ----
        self.declare_parameter('target_x', 0.5)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('angle_tolerance', 0.05)
        self.declare_parameter('max_linear_speed', 0.25)
        self.declare_parameter('max_angular_speed', 0.8)

        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        # 目标朝向同样归一化，避免传入 ±pi 之外的角度
        self.target_yaw = normalize_angle(self.get_parameter('target_yaw').value)
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value

        # 速度指令发布器（Gazebo 差速插件订阅 /cmd_vel）
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # 里程计反馈
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        # 50 Hz 控制周期
        self.create_timer(0.05, self._control_loop)

        self.x = None
        self.y = None
        self.yaw = None
        self.finished = False      # 是否已到达目标
        self._last_log_ns = 0      # 上一次打印进度日志的时间戳

        self.get_logger().info(
            f'Target pose: x={self.target_x:.3f}, y={self.target_y:.3f}, '
            f'yaw={self.target_yaw:.3f} rad'
        )
        self.get_logger().info('Waiting for /odom')

    def _odom_callback(self, message):
        """更新机器人当前位姿（odom 坐标系）。"""
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.x = position.x
        self.y = position.y
        # 由四元数反解偏航角（弧度）
        self.yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y
                          + orientation.z * orientation.z),
        )

    def _control_loop(self):
        """50 Hz 控制主循环：分两个阶段——接近目标位置、对准目标朝向。"""
        command = Twist()

        # 还没收到里程计或已到达：保持零速
        if self.x is None or self.finished:
            self.cmd_vel_publisher.publish(command)
            return

        distance = math.hypot(self.target_x - self.x, self.target_y - self.y)
        if distance > self.position_tolerance:
            # ---- 阶段 1：接近目标位置 ----
            target_heading = math.atan2(
                self.target_y - self.y,
                self.target_x - self.x,
            )
            heading_error = normalize_angle(target_heading - self.yaw)

            # 角速度 = 1.8 * 航向误差，夹在 ±max_angular_speed 内
            command.angular.z = clamp(
                1.8 * heading_error,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
            # 航向误差超过 0.6 rad 时先原地转向，误差足够小才允许前进；
            # 前进速度随剩余距离线性衰减（0.8 * distance），接近时减速
            if abs(heading_error) < 0.6:
                command.linear.x = clamp(
                    0.8 * distance,
                    0.0,
                    self.max_linear_speed,
                )
        else:
            # ---- 阶段 2：位置已到位，原地旋转对准目标朝向 ----
            angle_error = normalize_angle(self.target_yaw - self.yaw)
            if abs(angle_error) <= self.angle_tolerance:
                # 位置与朝向都收敛：到达目标
                self.finished = True
                self.get_logger().info(
                    f'Target reached: x={self.x:.3f}, y={self.y:.3f}, '
                    f'yaw={self.yaw:.3f}'
                )
            else:
                command.angular.z = clamp(
                    1.8 * angle_error,
                    -self.max_angular_speed,
                    self.max_angular_speed,
                )

        self._log_progress(distance)
        self.cmd_vel_publisher.publish(command)

    def _log_progress(self, distance):
        """每 2 秒打印一次当前距离与位姿，方便观察收敛过程。"""
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_log_ns < 2_000_000_000:
            return
        self._last_log_ns = now_ns
        self.get_logger().info(
            f'distance_to_target={distance:.3f}, '
            f'pose=({self.x:.3f}, {self.y:.3f}, {self.yaw:.3f})'
        )

    def stop(self):
        """退出时发布零速度，避免机器人继续运动。"""
        self.cmd_vel_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = PointController()
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
