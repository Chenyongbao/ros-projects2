#!/usr/bin/env python3
"""
统一安全门（学习阶段 7：由阶段 4 obstacle_stop 升级，所有命令源汇入单一安全门）

角色：
    /cmd_vel 全系统只有这一个发布者 —— 本节点。
    无论运动指令来自哪个控制器，都必须经过这里：
        Nav2（Jazzy bringup 内置 remap）  ── /cmd_vel_nav ──┐
        自定义控制器（阶段 5/6 等）         ── /cmd_vel_raw ──┼──> [本节点] ──> /cmd_vel ──> Gazebo
        急停 / 限速（service / topic）      ─────────────────┘

每个检查周期（默认 50 ms）按以下优先级决策（高→低）：
    1. emergency_stop  急停闩锁（/clear_emergency_stop 或 topic 发 false 才解除）
    2. stale_scan      雷达数据超时
    3. obstacle        正前方扇形内检测到障碍
    4. no_command      从未收到任何命令
    5. stale_command   命令看门狗超时
    6. forwarding      对最新命令限速后放行

命令源仲裁采用"最后写入优先"：/cmd_vel_raw 与 /cmd_vel_nav 两个来源中
时间戳更新的一个被放行，因此 Nav2 导航与教学控制器可以共存，无需额外 mux。

参数（可用 --ros-args -p 覆盖）：
    stop_distance   停止距离，默认 0.35 m
    stop_angle      前方检测扇形半角，默认 0.52 rad（约 ±30°）
    scan_timeout    雷达数据超时时间，默认 0.5 s
    command_timeout 命令看门狗超时时间，默认 0.5 s
    max_linear_speed  线速度上限，默认 0.0（0 = 不限制）
    max_angular_speed 角速度上限，默认 0.0（0 = 不限制）

话题：
    订阅 /scan（sensor_msgs/LaserScan，BEST_EFFORT QoS）
         /cmd_vel_raw、/cmd_vel_nav（geometry_msgs/Twist）
         /emergency_stop/command（std_msgs/Bool，true 触发、false 解除）
         /safety/linear_speed_limit（std_msgs/Float32，运行时线速上限，0 = 还原参数值）
    发布 /cmd_vel（唯一输出）
         /emergency_stop/state（std_msgs/Bool，当前急停状态）
         /safety_gate/state（std_msgs/String，当前决策原因，供监控与验收）

服务：
    /enable_emergency_stop、/clear_emergency_stop（std_srvs/SetBool）

用法：
    ros2 run neobot_control obstacle_stop
    # 与 Gazebo/Nav2 联跑时开启仿真时间：
    ros2 run neobot_control obstacle_stop --ros-args --params use_sim_time:=true
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool


class ObstacleStop(Node):
    """统一安全门：/cmd_vel 唯一发布者，含急停 / 限速 / 障碍物 / 看门狗。"""

    def __init__(self):
        super().__init__('obstacle_stop')

        # ---- 声明并读取参数 ----
        self.declare_parameter('stop_distance', 0.35)
        self.declare_parameter('stop_angle', 0.52)
        self.declare_parameter('scan_timeout', 0.5)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('max_linear_speed', 0.0)
        self.declare_parameter('max_angular_speed', 0.0)

        self.stop_distance = self.get_parameter('stop_distance').value
        self.stop_angle = self.get_parameter('stop_angle').value
        self.scan_timeout = self.get_parameter('scan_timeout').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value

        # 唯一输出：Gazebo 差速插件订阅 /cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # 监控 / 验收用状态话题
        self.estop_state_publisher = self.create_publisher(Bool, '/emergency_stop/state', 10)
        self.gate_state_publisher = self.create_publisher(String, '/safety_gate/state', 10)

        # 两个命令源：自定义控制器与 Nav2（Jazzy bringup 已把 Nav2 输出 remap 到 cmd_vel_nav）
        for topic in ('/cmd_vel_raw', '/cmd_vel_nav'):
            self.create_subscription(
                Twist, topic, lambda msg, t=topic: self._command_callback(t, msg), 10)
        # 激光雷达扫描数据（传感器话题用 BEST_EFFORT QoS）
        self.create_subscription(
            LaserScan, '/scan', self._scan_callback, qos_profile_sensor_data)
        # 急停命令与运行时限速
        self.create_subscription(
            Bool, '/emergency_stop/command', self._estop_command_callback, 10)
        self.create_subscription(
            Float32, '/safety/linear_speed_limit', self._speed_limit_callback, 10)

        self.enable_estop_service = self.create_service(
            SetBool, '/enable_emergency_stop', self._handle_enable_estop)
        self.clear_estop_service = self.create_service(
            SetBool, '/clear_emergency_stop', self._handle_clear_estop)

        # 50 ms 定时任务：持续发布安全决策
        self.create_timer(0.05, self._tick)

        # 命令源缓存：每个来源各自记录最近一条命令及其时间戳
        self.sources = {
            '/cmd_vel_raw': {'twist': Twist(), 'ns': 0, 'received': False},
            '/cmd_vel_nav': {'twist': Twist(), 'ns': 0, 'received': False},
        }
        self.last_scan_ns = 0            # 最后一帧雷达数据的时间戳（纳秒）
        self.scan_received = False       # 是否收到过雷达数据
        self.obstacle_detected = False   # 正前方是否检测到障碍物
        self.emergency_stop = False      # 急停闩锁
        self.runtime_linear_limit = 0.0  # 运行时线速上限；0.0 = 使用 max_linear_speed 参数
        self._last_state = None          # 上一次决策原因（用于状态变化日志）

        self.get_logger().info(
            f'Safety gate active: distance={self.stop_distance:.2f} m, '
            f'angle={self.stop_angle:.2f} rad, '
            f'limits linear={self.max_linear_speed:.2f} m/s, '
            f'angular={self.max_angular_speed:.2f} rad/s'
        )

    def _command_callback(self, topic, message):
        """缓存任一命令源的最近指令及时间戳（最后写入优先的仲裁依据）。"""
        source = self.sources[topic]
        source['twist'].linear.x = message.linear.x
        source['twist'].linear.y = message.linear.y
        source['twist'].linear.z = message.linear.z
        source['twist'].angular.x = message.angular.x
        source['twist'].angular.y = message.angular.y
        source['twist'].angular.z = message.angular.z
        source['ns'] = self.get_clock().now().nanoseconds
        source['received'] = True

    def _scan_callback(self, message):
        """
        扫描一帧雷达数据，判断正前方扇形内是否有近距障碍。

        对每一根射线：
            1. 跳过无效距离（非有限值或 <= 0）；
            2. 计算该射线相对机器人正前方的角度；
            3. 角度在 ±stop_angle 内且距离 <= stop_distance 时，判定有障碍物。
        """
        self.last_scan_ns = self.get_clock().now().nanoseconds
        self.scan_received = True
        self.obstacle_detected = False

        for index, range_value in enumerate(message.ranges):
            if not math.isfinite(range_value) or range_value <= 0.0:
                continue
            # 射线角度 = 起始角 + 索引 * 角分辨率，再归一化到 (-pi, pi]
            angle = message.angle_min + index * message.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= self.stop_angle and range_value <= self.stop_distance:
                self.obstacle_detected = True
                break

    def _estop_command_callback(self, message):
        """急停命令话题：true 触发急停，false 解除急停。"""
        self.emergency_stop = message.data
        if message.data:
            self.get_logger().warn('emergency stop enabled via topic')
        else:
            self.get_logger().info('emergency stop cleared via topic')

    def _speed_limit_callback(self, message):
        """运行时线速上限：大于 0 时覆盖参数值，0 时还原为参数值。"""
        self.runtime_linear_limit = max(0.0, message.data)

    def _handle_enable_estop(self, request, response):
        self.emergency_stop = True
        self.get_logger().warn('emergency stop enabled via service')
        response.success = True
        response.message = 'emergency stop active'
        return response

    def _handle_clear_estop(self, request, response):
        self.emergency_stop = False
        self.get_logger().info('emergency stop cleared via service')
        response.success = True
        response.message = 'emergency stop cleared'
        return response

    def _apply_speed_limit(self, twist):
        """放行前对命令做线速 / 角速限速（上限为 0 表示不限制）。"""
        linear_limit = (
            self.runtime_linear_limit if self.runtime_linear_limit > 0.0
            else self.max_linear_speed
        )
        if linear_limit > 0.0:
            linear_speed = math.hypot(twist.linear.x, twist.linear.y)
            if linear_speed > linear_limit and linear_speed > 0.0:
                scale = linear_limit / linear_speed
                twist.linear.x *= scale
                twist.linear.y *= scale
                twist.angular.z *= scale
        if self.max_angular_speed > 0.0:
            limit = self.max_angular_speed
            twist.angular.z = max(-limit, min(limit, twist.angular.z))
        return twist

    def _tick(self):
        """
        50 ms 定时任务：按优先级决策并发布安全后的速度指令。

        决策优先级（高→低）：
            急停 -> 雷达超时 -> 前方障碍 -> 无命令 -> 命令超时 -> 限速放行
        """
        now_ns = self.get_clock().now().nanoseconds
        # 雷达数据是否在 scan_timeout 内更新过
        scan_is_fresh = (
            self.scan_received
            and now_ns - self.last_scan_ns <= self.scan_timeout * 1e9
        )

        # 命令源仲裁：取"已收到且未超时"的来源中时间戳最新的一条
        active_source = None
        for topic, source in self.sources.items():
            if not source['received']:
                continue
            if now_ns - source['ns'] > self.command_timeout * 1e9:
                continue
            if active_source is None or source['ns'] > active_source[1]['ns']:
                active_source = (topic, source)

        if self.emergency_stop:
            reason = 'emergency_stop'
            command = Twist()
        elif not scan_is_fresh:
            reason = 'stale_scan'
            command = Twist()
        elif self.obstacle_detected:
            reason = 'obstacle'
            command = Twist()
        elif active_source is None:
            received_any = any(s['received'] for s in self.sources.values())
            reason = 'stale_command' if received_any else 'no_command'
            command = Twist()
        else:
            reason = 'forwarding:' + active_source[0]
            command = self._apply_speed_limit(active_source[1]['twist'])

        self.cmd_vel_publisher.publish(command)

        estop_state = Bool()
        estop_state.data = self.emergency_stop
        self.estop_state_publisher.publish(estop_state)
        gate_state = String()
        gate_state.data = reason
        self.gate_state_publisher.publish(gate_state)

        # 只在决策原因变化时打日志，避免刷屏
        if reason != self._last_state:
            self._last_state = reason
            if reason.startswith('forwarding:'):
                self.get_logger().info(
                    f'Path clear; forwarding {reason.split(":", 1)[1]}')
            else:
                self.get_logger().warn(f'Publishing zero velocity: {reason}')


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 退出前发一帧零速度，保证安全
        node.cmd_vel_publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
