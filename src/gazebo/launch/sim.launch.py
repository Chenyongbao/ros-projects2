# sim.launch.py
# Gazebo/Ignition 仿真环境启动脚本
# 负责启动 Ignition Gazebo 仿真世界、加载小车 SDF 环境模型，并通过 ros_gz_bridge 实现 ROS 2 与仿真引擎的话题桥接

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """生成并返回仿真环境与桥接节点启动描述"""
    gazebo_pkg_prefix = get_package_share_directory('gazebo')
    gazebo_sim_ign = os.path.join(gazebo_pkg_prefix, 'launch', 'sim.ign')
    sdf_file_path = os.path.join(gazebo_pkg_prefix, 'launch', 'robot_env.sdf')
    
    # 1. 启动 Ignition Gazebo 仿真渲染界面
    gz_sim = ExecuteProcess(cmd=['ign', 'launch', '-v 4', f'{gazebo_sim_ign}'])
    
    # 2. 启动 Ignition Gazebo 仿真物理服务端并加载机器人与赛道 SDF 模型
    gz_sim_server = ExecuteProcess(cmd=['ign', 'gazebo', '-s', '-v 4', '-r', f'{sdf_file_path}'])

    # 3. 配置 ROS-GZ Bridge 桥接节点
    # 双向或单向桥接 TF 坐标树、底盘控制速度(/cmd_vel)、IMU(/imu)、激光雷达(/lidar)、里程计与相机图像
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/robot/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/model/robot/pose_static@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/model/robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'
        ],
        parameters=[{'qos_overrides./model/vehicle_blue.subscriber.reliability': 'reliable'}],
        output='screen',
        remappings=[
            ('/model/robot/pose', '/tf'),
            ('/model/robot/pose_static', '/tf')
        ]
    )

    return LaunchDescription([
        gz_sim,
        gz_sim_server,
        bridge
    ])