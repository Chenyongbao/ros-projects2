# setup.py
# ROS 2 Python 包 aggregator 的打包与安装配置文件

import os
from glob import glob
from setuptools import setup

package_name = 'aggregator'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # 1. 注册 ament 资源索引
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # 2. 安装 package.xml 描述文件
        (os.path.join('share', package_name), ['package.xml']),
        # 3. 安装 launch 启动脚本
        (os.path.join('share', package_name, 'launch'), \
         glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eddyzhou, aryanafrouzi',
    maintainer_email='e23zhou@watonomous.ca, aryanafrouzi@swaprobotics.com',
    description='聚合器示例包：统计各话题消息数量与频率并在控制台输出',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        # 声明终端可执行节点入口
        'console_scripts': [
            'aggregator_node = aggregator.aggregator_node:main'
        ],
    },
)

