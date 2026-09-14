from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch 文件只负责启动节点，不把业务逻辑写进启动脚本。
    return LaunchDescription([
        # 通过 LaunchConfiguration 把命令行参数传给两个独立节点。
        DeclareLaunchArgument("max_retries", default_value="1"),
        DeclareLaunchArgument("fail_first_goal", default_value="false"),
        DeclareLaunchArgument("auto_dispatch", default_value="false"),
        DeclareLaunchArgument("use_mock_navigation", default_value="true"),
        Node(
            # package/executable 必须与 learn_order_node 的 CMake 安装目标一致。
            package="learn_order_node",
            executable="order_gateway_node",
            name="order_gateway_node",
            output="screen",
            parameters=[{
                "max_retries": LaunchConfiguration("max_retries"),
                "auto_dispatch": LaunchConfiguration("auto_dispatch"),
            }],
        ),
        Node(
            package="learn_order_node",
            executable="mock_navigation_server",
            name="mock_navigation_server",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_mock_navigation")),
            parameters=[{"fail_first_goal": LaunchConfiguration("fail_first_goal")}],
        ),
        Node(
            package="learn_order_node",
            executable="nav2_navigation_server",
            name="nav2_navigation_server",
            output="screen",
            condition=UnlessCondition(LaunchConfiguration("use_mock_navigation")),
        ),
    ])
