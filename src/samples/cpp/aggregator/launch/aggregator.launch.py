# aggregator.launch.py
# 聚合器节点启动脚本，负责启动 aggregator_node

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """生成并返回聚合器节点启动描述"""
    aggregator_node = Node(
        package='aggregator',
        executable='aggregator_node',
        output='screen'
    )

    return LaunchDescription([
        aggregator_node
    ])

