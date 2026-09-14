import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 仿真机器人名称必须与 Gazebo 中的实体名称一致。
    robot_name_in_model = 'neobot'
    package_name = 'neobot_description'
    urdf_name = "neobot_gazebo.urdf"
    use_gui = LaunchConfiguration('gui')

    ld = LaunchDescription()
    # 从安装后的功能包目录获取 URDF 和 Gazebo 世界文件。
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')
    gazebo_world_path = os.path.join(pkg_share, 'world/neobot.world')

    # 启动 Gazebo 图形客户端和服务器。没有 DISPLAY 时使用 gui:=false。
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        condition=IfCondition(use_gui),
        output='screen')

    # 无图形界面时只启动服务器，适合 WSL、SSH 和 CI 环境。
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', gazebo_world_path],
        condition=UnlessCondition(use_gui),
        output='screen')

    # 将 URDF 文件生成到当前 Gazebo 世界中，并抬高 2 cm，避免轮子和地面初始穿透。
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name_in_model,
            '-file', urdf_model_path,
            '-z', '0.02',
        ],
        output='screen')
    
    # 根据 URDF 发布机器人各个 link 之间的 TF。
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
    )

    # 预留 RViz 启动配置；当前默认只启动 Gazebo，不自动启动 RViz。
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # arguments=['-d', default_rviz_config_path]
        )

    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    #ld.add_action(start_rviz_cmd) 注释完即launch的时候不启动rviz

    return LaunchDescription([
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='是否启动 Gazebo 图形界面；无 DISPLAY 时设为 false'),
        *ld.entities,
    ])
