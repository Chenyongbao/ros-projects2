# robot.launch.py
# 避障自主导航小车的主启动 Launch 文件
# 负责依次启动局部代价地图(costmap)、全局地图记忆(map_memory)、A*路径规划(planner)、底盘跟踪控制(control)以及伪里程计(odometry_spoof)五个核心节点

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """生成并返回 ROS 2 机器人完整运行环境启动描述"""
    ld = LaunchDescription()

    # ==================== 1. Costmap 局部代价地图节点 ====================
    costmap_pkg_prefix = get_package_share_directory('costmap')
    costmap_param_file = os.path.join(costmap_pkg_prefix, 'config', 'params.yaml')
    
    costmap_param = DeclareLaunchArgument(
        'costmap_param_file',
        default_value=costmap_param_file,
        description='局部代价地图节点的配置文件路径'
    )
    costmap_node = Node(
        package='costmap',
        name='costmap_node',
        executable='costmap_node',
        parameters=[LaunchConfiguration('costmap_param_file')],
        output='screen'
    )
    ld.add_action(costmap_param)
    ld.add_action(costmap_node)

    # ==================== 2. Map Memory 地图记忆与融合节点 ====================
    map_memory_pkg_prefix = get_package_share_directory('map_memory')
    map_memory_param_file = os.path.join(map_memory_pkg_prefix, 'config', 'params.yaml')
    
    map_memory_param = DeclareLaunchArgument(
        'map_memory_param_file',
        default_value=map_memory_param_file,
        description='地图记忆与融合节点的配置文件路径'
    )
    map_memory_node = Node(
        package='map_memory',
        name='map_memory_node',
        executable='map_memory_node',
        parameters=[LaunchConfiguration('map_memory_param_file')],
        output='screen'
    )
    ld.add_action(map_memory_param)
    ld.add_action(map_memory_node)
    
    # ==================== 3. Planner 路径规划节点 ====================
    planner_pkg_prefix = get_package_share_directory('planner')
    planner_param_file = os.path.join(planner_pkg_prefix, 'config', 'params.yaml')
    
    planner_param = DeclareLaunchArgument(
        'planner_param_file',
        default_value=planner_param_file,
        description='A* 路径规划节点的配置文件路径'
    )
    planner_node = Node(
        package='planner',
        name='planner_node',
        executable='planner_node',
        parameters=[LaunchConfiguration('planner_param_file')],
        output='screen'
    )
    ld.add_action(planner_param)
    ld.add_action(planner_node)
    
    # ==================== 4. Control 底盘运动控制节点 ====================
    control_pkg_prefix = get_package_share_directory('control')
    control_param_file = os.path.join(control_pkg_prefix, 'config', 'params.yaml')
    
    control_param = DeclareLaunchArgument(
        'control_param_file',
        default_value=control_param_file,
        description='底盘纯追踪跟踪控制节点的配置文件路径'
    )
    control_node = Node(
        package='control',
        name='control_node',
        executable='control_node',
        parameters=[LaunchConfiguration('control_param_file')],
        output='screen'
    )
    ld.add_action(control_param)
    ld.add_action(control_node)

    # ==================== 5. Odometry Spoof 里程计伪造转换节点 ====================
    odometry_spoof_node = Node(
        package='odometry_spoof',
        name='odometry_spoof',
        executable='odometry_spoof',
        output='screen'
    )
    ld.add_action(odometry_spoof_node)

    return ld

