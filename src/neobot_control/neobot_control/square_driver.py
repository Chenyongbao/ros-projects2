#!/usr/bin/env python3
"""
正方形驾驶节点（学习阶段 2：第一个"闭环"控制节点）

功能：
    以收到第一帧 /odom 时的位姿为起点，让机器人沿一个正方形行驶一圈。
    内部是一个简单的状态机，对每一条边交替执行：
        drive —— 直线行驶到本边的终点（目标角）；
        turn  —— 原地旋转 90°，对准下一条边的方向。
    四条边全部完成后节点输出零速度并提示完成。

参数（可用 --ros-args -p 覆盖）：
    side_length        正方形边长，默认 1.0 m
    linear_speed       直线行驶速度，默认 0.2 m/s
    angular_speed      转向速度，默认 0.5 rad/s
    position_tolerance 到达边终点的距离容差，默认 0.05 m
    angle_tolerance    到达目标朝向的角度容差，默认 0.05 rad

用法：
    ros2 run neobot_control square_driver --ros-args -p side_length:=1.0

注意：
    运行前请停止 teleop / Nav2 等同样发布 /cmd_vel 的节点，避免指令冲突。
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def normalize_angle(angle):
    """把任意角度归一化到 (-pi, pi] 区间，避免角度差跨 ±pi 出错。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lower, upper):
    """把数值夹在 [lower, upper] 区间内。"""
    return max(lower, min(value, upper))


class SquareDriver(Node):
    """按"直行—转弯"交替的有限状态机，驱动机器人走完一个正方形。"""

    def __init__(self):
        super().__init__('square_driver')

        # ---- 声明并读取参数 ----
        self.declare_parameter('side_length', 1.0)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('angle_tolerance', 0.05)

        self.side_length = self.get_parameter('side_length').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value

        # 速度指令发布器：Gazebo 差速插件订阅 /cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # 订阅里程计，作为闭环反馈
        self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10,
        )
        # 50 Hz 控制周期
        self.create_timer(0.05, self._control_loop)

        # ---- 运行时状态 ----
        self.x = None
        self.y = None
        self.yaw = None
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.side_index = 0      # 当前正在处理的第几条边（0 ~ 3）
        self.state = 'drive'     # 状态机：'drive' 直行 / 'turn' 原地转向
        self.target_x = None     # 当前边终点的 x
        self.target_y = None     # 当前边终点的 y
        self.target_yaw = None   # 转向状态下的目标朝向
        self.finished = False    # 正方形是否已走完

        self.get_logger().info('Waiting for /odom before starting the square')

    def _odom_callback(self, message):
        """更新机器人当前位姿；收到第一帧里程计时确定正方形起点。"""
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

        # 第一帧里程计：把当前位姿记为正方形起点，并设置第一条边的目标
        if self.start_x is None:
            self.start_x = self.x
            self.start_y = self.y
            self.start_yaw = self.yaw
            self._set_drive_target()
            self.get_logger().info(
                f'Square start: x={self.start_x:.3f}, y={self.start_y:.3f}, '
                f'yaw={self.start_yaw:.3f}'
            )

    def _set_drive_target(self):
        """
        计算第 side_index 条边终点的目标位置与朝向。

        从起点开始，沿每条边的朝向（起点朝向依次 +90°）累加
        side_length 的位移向量，即可得到当前边终点坐标。
        """
        self.target_x = self.start_x
        self.target_y = self.start_y
        for segment in range(self.side_index + 1):
            heading = self.start_yaw + segment * math.pi / 2.0
            self.target_x += self.side_length * math.cos(heading)
            self.target_y += self.side_length * math.sin(heading)
        self.target_yaw = normalize_angle(heading)
        self.state = 'drive'

    def _control_loop(self):
        """50 Hz 控制主循环：根据当前状态机状态计算并发布速度指令。"""
        command = Twist()

        # 还没收到里程计，或已经完成：保持零速
        if self.x is None or self.finished:
            self.cmd_vel_publisher.publish(command)
            return

        if self.state == 'drive':
            # ---- 直行阶段：朝本边终点直线行驶 ----
            distance = math.hypot(self.target_x - self.x, self.y - self.y) \
                if False else math.hypot(self.target_x - self.x, self.target_y - self.y)
            if distance <= self.position_tolerance:
                # 到达边终点：进入转向阶段，目标朝向 = 起点朝向 + (边数+1)*90°
                self.state = 'turn'
                self.target_yaw = normalize_angle(
                    self.start_yaw + (self.side_index + 1) * math.pi / 2.0
                )
                self.get_logger().info(
                    f'Side {self.side_index + 1} reached; turning to '
                    f'yaw={self.target_yaw:.3f}'
                )
            else:
                # 朝目标点方向行驶：航向误差 = 目标方向 - 当前朝向
                target_heading = math.atan2(
                    self.target_y - self.y,
                    self.target_x - self.x,
                )
                heading_error = normalize_angle(target_heading - self.yaw)
                # 线性速度：越接近终点越慢（0.5 * 剩余距离），防止过冲
                command.linear.x = min(self.linear_speed, 0.5 * distance)
                # 角速度：按航向误差的比例控制，限制在 ±0.5 rad/s 内
                command.angular.z = clamp(1.5 * heading_error, -0.5, 0.5)

        if self.state == 'turn':
            # ---- 原地转向阶段：转到下一条边的朝向 ----
            angle_error = normalize_angle(self.target_yaw - self.yaw)
            if abs(angle_error) <= self.angle_tolerance:
                # 转向完成，进入下一条边；4 条边全部完成则结束
                self.side_index += 1
                if self.side_index >= 4:
                    self.finished = True
                    self.get_logger().info('Square completed')
                else:
                    self._set_drive_target()
                    self.get_logger().info(
                        f'Starting side {self.side_index + 1}'
                    )
            else:
                # 按比例转向，速度上限 angular_speed；方向由误差符号决定
                command.angular.z = math.copysign(
                    min(self.angular_speed, 1.5 * abs(angle_error)),
                    angle_error,
                )

        self.cmd_vel_publisher.publish(command)

    def stop(self):
        """退出时发布零速度，避免机器人以最后一条指令继续运动。"""
        self.cmd_vel_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
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
