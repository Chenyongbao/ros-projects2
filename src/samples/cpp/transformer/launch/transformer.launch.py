# transformer.launch.py
# 转换器节点启动脚本，加载 params.yaml 配置文件并启动 transformer_node

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成并返回转换器节点启动描述"""
    transformer_pkg_prefix = get_package_share_directory('transformer')
    transformer_param_file = os.path.join(
        transformer_pkg_prefix, 'config', 'params.yaml')

    # 声明配置文件路径参数
    transformer_param = DeclareLaunchArgument(
        'transformer_param_file',
        default_value=transformer_param_file,
        description='转换器节点的配置文件路径'
    )

    # 启动转换器节点
    transformer_node = Node(
        package='transformer',
        executable='transformer_node',
        parameters=[LaunchConfiguration('transformer_param_file')],
        output='screen'
    )

    return LaunchDescription([
        transformer_param,
        transformer_node
    ])

