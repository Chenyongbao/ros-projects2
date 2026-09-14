#!/usr/bin/env python3
"""
Nav2 导航目标客户端（学习阶段 5：调用 Nav2 Action 完成自主导航）

功能：
    通过 Nav2 的 /navigate_to_pose Action 接口发送一个目标位姿，
    接收目标接受/拒绝响应、持续的距离反馈以及最终成功/失败结果。
    本节点本身不直接控制机器人速度，所有运动由 Nav2 内部的
    controller_server 完成。

    Action 通信流程：
        1. 客户端发送 goal（目标位姿 + 坐标系）
        2. 服务端返回 goal_handle（接受/拒绝）
        3. 服务端持续发布 feedback（剩余距离、恢复次数）
        4. 服务端返回 result（成功/取消/失败）

参数（可用 --ros-args -p 覆盖）：
    target_x     目标 x 坐标（米），默认 0.5
    target_y     目标 y 坐标（米），默认 0.0
    target_yaw   目标朝向（弧度），默认 0.0
    frame_id     目标所在的坐标系，默认 'map'

用法：
    ros2 run neobot_control navigate_to_pose --ros-args \\
      -p target_x:=0.5 -p target_y:=0.3 -p target_yaw:=1.57

注意：
    运行前需要先启动 Gazebo 和 Nav2 栈。
    运行期间不要同时启动 point_controller / square_driver 等
    同样发布 /cmd_vel 的自定义节点。
"""

import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class NavigateToPoseClient(Node):
    """Nav2 NavigateToPose Action 客户端：发送目标并监听结果。"""

    def __init__(self):
        # 注册节点，节点名为 'navigate_to_pose_client'
        super().__init__('navigate_to_pose_client')

        # ---- 声明并读取参数 ----
        self.declare_parameter('target_x', 0.5)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('frame_id', 'map')

        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.target_yaw = self.get_parameter('target_yaw').value
        self.frame_id = self.get_parameter('frame_id').value

        # 创建 Action 客户端，连接 Nav2 的 /navigate_to_pose 服务端
        self.action_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose',
        )
        # 反馈日志限频：上次打印反馈的时间戳（纳秒）
        self._last_feedback_ns = 0
        # 完成后自动关闭节点的定时器
        self._shutdown_timer = self.create_timer(0.1, self._try_shutdown)
        self._finished = False  # 目标是否已完成（成功/失败/取消）

        self.get_logger().info(
            f'Navigation goal: frame={self.frame_id}, '
            f'x={self.target_x:.3f}, y={self.target_y:.3f}, '
            f'yaw={self.target_yaw:.3f} rad'
        )
        # 启动后延迟 0.1s 发送目标，等待 Action Server 就绪
        self._send_goal_timer = self.create_timer(0.1, self._send_goal)

    def _send_goal(self):
        """构造并异步发送导航目标，等待服务端响应。"""
        self._send_goal_timer.cancel()
        self.get_logger().info('Waiting for Nav2 /navigate_to_pose action server')
        # 等待 Action Server 可用，最多 10 秒；超时则退出
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server is not available')
            self._finished = True
            return

        # 构造目标消息：在指定坐标系中放置一个目标位姿
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.target_x
        goal.pose.pose.position.y = self.target_y
        # 偏航角转四元数：只用 Z 轴旋转，x=y=0
        goal.pose.pose.orientation.z = math.sin(self.target_yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.target_yaw / 2.0)

        # 异步发送目标，同时注册反馈回调
        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback,
        )
        future.add_done_callback(self._goal_response_callback)
        self.get_logger().info('Navigation goal sent')

    def _goal_response_callback(self, future):
        """处理服务端对目标的接受/拒绝响应。"""
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f'Failed to send navigation goal: {error}')
            self._finished = True
            return

        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal was rejected')
            self._finished = True
            return

        self.get_logger().info('Navigation goal accepted')
        # 目标被接受后，注册结果回调以等待最终状态
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_message):
        """接收 Nav2 过程反馈：剩余距离和恢复次数。"""
        feedback = feedback_message.feedback
        now_ns = self.get_clock().now().nanoseconds
        # 每 2 秒最多打印一次反馈，避免刷屏
        if now_ns - self._last_feedback_ns < 2_000_000_000:
            return
        self._last_feedback_ns = now_ns
        self.get_logger().info(
            f'Nav2 feedback: distance_remaining='
            f'{feedback.distance_remaining:.3f} m, '
            f'recoveries={feedback.number_of_recoveries}'
        )

    def _result_callback(self, future):
        """接收最终结果：成功、取消或失败。"""
        try:
            status = future.result().status
        except Exception as error:
            self.get_logger().error(f'Failed to receive navigation result: {error}')
            self._finished = True
            return

        # 根据 GoalStatus 枚举判断结果类型
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation goal succeeded')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation goal canceled')
        else:
            self.get_logger().error(f'Navigation goal failed with status {status}')
        self._finished = True

    def _try_shutdown(self):
        """目标完成后自动关闭节点。"""
        if self._finished:
            self._shutdown_timer.cancel()
            rclpy.shutdown()


def main(args=None):
    """节点入口：初始化 rclpy，创建客户端节点，保持运行直到目标完成。"""
    rclpy.init(args=args)
    node = NavigateToPoseClient()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
