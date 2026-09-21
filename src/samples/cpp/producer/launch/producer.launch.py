# producer.launch.py
# 生产者节点启动脚本，加载 params.yaml 配置文件并启动 producer_node

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成并返回生产者节点启动描述"""
    producer_pkg_prefix = get_package_share_directory('producer')
    producer_param_file = os.path.join(
        producer_pkg_prefix, 'config', 'params.yaml')

    # 声明参数文件路径参数
    producer_param = DeclareLaunchArgument(
        'producer_param_file',
        default_value=producer_param_file,
        description='生产者节点配置文件路径'
    )

    # 声明并配置节点
    producer_node = Node(
        package='producer',
        name='producer_node',
        executable='producer_node',
        parameters=[LaunchConfiguration('producer_param_file')],
        output='screen'
    )

    return LaunchDescription([
        producer_param,
        producer_node
    ])

