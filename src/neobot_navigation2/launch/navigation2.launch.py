# nav2 launch file

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    #=============================1.定位到包的地址=============================================================
    neobot_navigation2_dir = get_package_share_directory('neobot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')


    #=============================2.声明参数，获取配置文件路径===================================================
    # use_sim_time 这里要设置成true,因为gazebo是仿真环境，其时间是通过/clock话题获取，而不是系统时间
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_path = LaunchConfiguration('map',default=os.path.join(neobot_navigation2_dir,'maps','neobot_map.yaml'))
    nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(neobot_navigation2_dir,'param','neobot_nav2.yaml'))
    # 使用本项目的 RViz 配置，而不是 nav2_bringup 的系统默认配置。
    rviz_config_dir = os.path.join(neobot_navigation2_dir, 'rviz', 'neobot_navigation2.rviz')
    # 是否启用统一安全门。Jazzy 的 nav2_bringup 已把 Nav2 控制器输出 remap 到 /cmd_vel_nav，
    # 安全门订阅该话题并经急停/障碍/看门狗检查后，作为 /cmd_vel 唯一发布者输出给底盘。
    use_safety_gate = LaunchConfiguration('use_safety_gate', default='true')

    #=============================3.声明启动launch文件，传入：地图路径、是否使用仿真时间以及nav2参数文件==============
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value=use_sim_time,description='Use simulation (Gzabo) clock if true'),
        DeclareLaunchArgument('map',default_value=map_yaml_path,description='Full path to map file to load'),
        DeclareLaunchArgument('params_file',default_value=nav2_param_path,description='Full path to nav2 param file to load'),
        DeclareLaunchArgument('use_safety_gate',default_value=use_safety_gate,
                              description='启用统一安全门：/cmd_vel 唯一发布者（急停/障碍/看门狗）'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir,'/launch','/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        # 统一安全门（C++ 节点，neobot_teleop 包）：订阅 /cmd_vel_nav（Nav2 输出）
        # 与 /cmd_vel_raw（自定义控制器），经急停/雷达/看门狗检查后作为 /cmd_vel
        # 唯一发布者输出给底盘。
        Node(
            package='neobot_teleop',
            executable='cmd_vel_safety_gate',
            name='safety_gate',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_safety_gate),
        ),
        Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_dir],
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'),
    ])
