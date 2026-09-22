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
    #类似：td::vector<Action> ld; // 创建一个空的任务列表容器
    ld = LaunchDescription()

    # ==================== 1. Costmap 局部代价地图节点 ====================
    #generate_launch_description(): 构造函数
    costmap_pkg_prefix = get_package_share_directory('costmap')

    #params.yaml:允许外部传入新的路径
    costmap_param_file = os.path.join(costmap_pkg_prefix, 'config', 'params.yaml')
    
    #构造函数中的参数
    #相当于带默认实参的函数形参
    #void launchRobot(std::string costmap_param_file = "/default/path/to/params.yaml")
    costmap_param = DeclareLaunchArgument(
        'costmap_param_file',
        default_value=costmap_param_file,
        description='局部代价地图节点的配置文件路径'
    )
    costmap_node = Node(
        package='costmap',
        name='costmap_node',
        executable='costmap_node',
        #LaunchConfiguration:相当于在构造函数内部引用这些参数变量
        parameters=[LaunchConfiguration('costmap_param_file')],
        output='screen'
    )
    #把所有配置好的模块装配进系统容器中。
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

    # ==================== 6. Mission Manager 多点任务管理节点 ====================
    # 任务队列状态机（IDLE/ACTIVE/RETRY）+ 重试跳过策略 + 任务状态发布
    mission_manager_node = Node(
        package='mission_manager',
        name='mission_manager',
        executable='mission_manager_node',
        output='screen'
    )
    ld.add_action(mission_manager_node)

    # ==================== 7. Sensor Simulator 传感器噪声仿真节点 ====================
    # 按 Thrun 速度运动模型注噪：/odom_raw → /wheel_odom（漂移位姿）+ /imu/data（噪声角速度）
    sensor_sim_pkg_prefix = get_package_share_directory('sensor_simulator')
    sensor_sim_param_file = os.path.join(sensor_sim_pkg_prefix, 'config', 'params.yaml')

    sensor_sim_param = DeclareLaunchArgument(
        'sensor_sim_param_file',
        default_value=sensor_sim_param_file,
        description='传感器噪声仿真节点的配置文件路径'
    )
    sensor_sim_node = Node(
        package='sensor_simulator',
        name='sensor_simulator',
        executable='sensor_simulator',
        parameters=[LaunchConfiguration('sensor_sim_param_file')],
        output='screen'
    )
    ld.add_action(sensor_sim_param)
    ld.add_action(sensor_sim_node)

    # ==================== 8. EKF Localizer 定位融合节点 ====================
    # 融合带噪轮速里程计与 IMU，发布 /odom/filtered（下游接口零改动）
    ekf_pkg_prefix = get_package_share_directory('ekf_localizer')
    ekf_param_file = os.path.join(ekf_pkg_prefix, 'config', 'params.yaml')

    ekf_param = DeclareLaunchArgument(
        'ekf_param_file',
        default_value=ekf_param_file,
        description='EKF 定位融合节点的配置文件路径'
    )
    ekf_node = Node(
        package='ekf_localizer',
        name='ekf_localizer',
        executable='ekf_node',
        parameters=[LaunchConfiguration('ekf_param_file')],
        output='screen'
    )
    ld.add_action(ekf_param)
    ld.add_action(ekf_node)

    return ld

