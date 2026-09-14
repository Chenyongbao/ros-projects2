#!/usr/bin/env python3
"""
安全门验收驱动（配合 scripts/check_safety_gate.sh 使用）

直接拉起 cmd_vel_safety_gate 安全门节点（C++，neobot_teleop 包），
喂入合成的 /scan 与命令消息，验证全部决策分支，
不需要 Gazebo 或 Nav2，约 15 秒完成：

    1  无雷达数据            -> stale_scan，零速
    2  雷达正常、无命令       -> no_command，零速
    3  自定义命令 0.5 m/s    -> forwarding:/cmd_vel_raw 直通
    4  Nav2 命令 0.3 m/s     -> 仲裁后 forwarding:/cmd_vel_nav
    5  正前方障碍            -> obstacle，零速
    6  雷达断开              -> stale_scan，零速
    7  雷达恢复              -> forwarding 恢复
    8  命令全部中断          -> stale_command，零速
    9  急停（service）       -> emergency_stop，零速
    10 解除急停             -> forwarding 恢复
    11 运行时限速 0.25 m/s   -> /cmd_vel 被钳制
    12 限速还原              -> 恢复原速

用法：
    bash scripts/check_safety_gate.sh
"""

import math
import os
import shutil
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32, String

RAW_SPEED = 0.5   # 自定义命令源速度（m/s）
NAV_SPEED = 0.3   # Nav2 命令源速度（m/s）
LIMIT = 0.25      # 运行时限速值（m/s）


class CheckNode(Node):
    """驱动节点：发布合成命令 / 雷达帧，记录安全门输出状态。"""

    def __init__(self):
        super().__init__('safety_gate_check')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.nav_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.scan_pub = self.create_publisher(
            LaserScan, '/scan', qos_profile_sensor_data)
        self.limit_pub = self.create_publisher(
            Float32, '/safety/linear_speed_limit', 10)

        self.gate_state = None     # 最新 /safety_gate/state
        self.cmd_vel = Twist()     # 最新 /cmd_vel
        self.cmd_seen = False      # 是否收到过 /cmd_vel
        self.estop_state = None    # 最新 /emergency_stop/state
        self.create_subscription(String, '/safety_gate/state', self._on_gate_state, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(Bool, '/emergency_stop/state', self._on_estop, 10)

        self.raw_active = False
        self.nav_active = False
        self.scan_active = False
        self.scan_obstacle = False
        self.create_timer(0.05, self._publish_periodic)

    def _on_gate_state(self, message):
        self.gate_state = message.data

    def _on_cmd_vel(self, message):
        self.cmd_vel = message
        self.cmd_seen = True

    def _on_estop(self, message):
        self.estop_state = message.data

    def _publish_periodic(self):
        """20 Hz：按开关状态发布合成命令与雷达帧。"""
        if self.raw_active:
            cmd = Twist()
            cmd.linear.x = RAW_SPEED
            self.cmd_pub.publish(cmd)
        if self.nav_active:
            cmd = Twist()
            cmd.linear.x = NAV_SPEED
            self.nav_pub.publish(cmd)
        if self.scan_active:
            self.scan_pub.publish(self.make_scan(self.scan_obstacle))

    def make_scan(self, obstacle):
        """361 根射线（-pi ~ pi）；正前方 0° 射线在 obstacle 时为 0.25 m。"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'laser_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0
        scan.ranges = [10.0] * 361
        scan.intensities = [0.0] * 361
        if obstacle:
            scan.ranges[180] = 0.25
        return scan


def main():
    if shutil.which('ros2') is None:
        sys.exit('找不到 ros2，请先 source ROS 2 环境')
    # ros2 run 通过 ament 索引解析可执行文件，不要求其在 PATH 上
    prefix_result = subprocess.run(
        ['ros2', 'pkg', 'prefix', 'neobot_teleop'],
        capture_output=True, text=True, timeout=15)
    package_prefix = prefix_result.stdout.strip()
    executable = os.path.join(
        package_prefix, 'lib', 'neobot_teleop', 'cmd_vel_safety_gate')
    if prefix_result.returncode != 0 or not os.path.isfile(executable):
        sys.exit(
            '找不到 neobot_teleop 包的 cmd_vel_safety_gate 可执行文件，'
            '请先 colcon build 并 source install/setup.bash')

    gate_log = tempfile.NamedTemporaryFile('w', suffix='.log', delete=False)
    gate = subprocess.Popen(
        ['ros2', 'run', 'neobot_teleop', 'cmd_vel_safety_gate'],
        stdout=gate_log, stderr=subprocess.STDOUT)

    def gate_log_tail():
        gate_log.flush()
        with open(gate_log.name, 'r', errors='replace') as fh:
            lines = fh.readlines()
        return ''.join(lines[-15:])

    rclpy.init()
    node = CheckNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def expect_state(expected, timeout=6.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)
            if node.gate_state == expected:
                return
        raise AssertionError(
            f'期望安全门状态 "{expected}"，实际 "{node.gate_state}"')

    def expect_cmd(x, tol=0.05, timeout=6.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)
            if node.cmd_seen and abs(node.cmd_vel.linear.x - x) <= tol:
                return
        raise AssertionError(
            f'期望 /cmd_vel linear.x ≈ {x}，实际 {node.cmd_vel.linear.x}')

    def expect_estop(expected, timeout=6.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.05)
            if node.estop_state is expected:
                return
        raise AssertionError(
            f'期望 /emergency_stop/state={expected}，实际 {node.estop_state}')

    def call_service(service_name, data):
        result = subprocess.run(
            ['ros2', 'service', 'call', service_name, 'std_srvs/srv/SetBool',
             '{data: %s}' % str(data).lower()],
            capture_output=True, text=True, timeout=15)
        if 'success=True' not in result.stdout:
            raise AssertionError(
                f'服务 {service_name} 调用失败：{result.stdout} {result.stderr}')

    try:
        print('等待安全门启动 ...')
        end = time.monotonic() + 15
        while node.gate_state is None:
            executor.spin_once(timeout_sec=0.1)
            if gate.poll() is not None:
                raise RuntimeError('安全门进程提前退出：\n' + gate_log_tail())
            if time.monotonic() > end:
                raise TimeoutError('安全门 15 秒内未启动')

        print('1.  无雷达数据         -> stale_scan，零速')
        expect_state('stale_scan')
        expect_cmd(0.0, tol=0.01)

        print('2.  雷达正常、无命令   -> no_command，零速')
        node.scan_active = True
        expect_state('no_command')
        expect_cmd(0.0, tol=0.01)

        print('3.  自定义命令 0.5 m/s -> forwarding:/cmd_vel_raw 直通')
        node.raw_active = True
        expect_state('forwarding:/cmd_vel_raw')
        expect_cmd(RAW_SPEED)

        print('4.  Nav2 命令 0.3 m/s  -> 仲裁 -> forwarding:/cmd_vel_nav')
        node.raw_active = False
        node.nav_active = True
        expect_state('forwarding:/cmd_vel_nav')
        expect_cmd(NAV_SPEED)

        print('5.  正前方障碍         -> obstacle，零速')
        node.scan_obstacle = True
        expect_state('obstacle')
        expect_cmd(0.0, tol=0.01)

        print('6.  雷达断开           -> stale_scan，零速')
        node.scan_active = False
        expect_state('stale_scan')
        expect_cmd(0.0, tol=0.01)

        print('7.  雷达恢复           -> forwarding:/cmd_vel_nav 恢复')
        node.scan_obstacle = False
        node.scan_active = True
        expect_state('forwarding:/cmd_vel_nav')
        expect_cmd(NAV_SPEED)

        print('8.  命令全部中断       -> stale_command，零速')
        node.nav_active = False
        expect_state('stale_command')
        expect_cmd(0.0, tol=0.01)

        print('9.  急停               -> emergency_stop，零速')
        node.raw_active = True
        call_service('/enable_emergency_stop', True)
        expect_state('emergency_stop')
        expect_cmd(0.0, tol=0.01)
        expect_estop(True)

        print('10. 解除急停           -> forwarding:/cmd_vel_raw 恢复')
        call_service('/clear_emergency_stop', True)
        expect_state('forwarding:/cmd_vel_raw')
        expect_cmd(RAW_SPEED)
        expect_estop(False)

        print('11. 运行时限速 %.2f    -> /cmd_vel 被钳制' % LIMIT)
        node.limit_pub.publish(Float32(data=LIMIT))
        expect_cmd(LIMIT)

        print('12. 限速还原           -> 恢复 %.2f m/s' % RAW_SPEED)
        node.limit_pub.publish(Float32(data=0.0))
        expect_cmd(RAW_SPEED)

        print('\nPASS：安全门 12 项验收全部通过')
    except (AssertionError, RuntimeError, TimeoutError) as exc:
        sys.exit('FAIL：%s\n--- 安全门日志尾部 ---\n%s' % (exc, gate_log_tail()))
    finally:
        gate.terminate()
        try:
            gate.wait(timeout=5)
        except subprocess.TimeoutExpired:
            gate.kill()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
