# setup.py
# ROS 2 Python 包 producer 的打包与安装配置文件

import os
from glob import glob
from setuptools import setup

package_name = 'producer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # 1. 在 ament 索引中注册当前包
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # 2. 安装 package.xml 元数据文件到 share 目录
        (os.path.join('share', package_name), ['package.xml']),
        # 3. 安装 launch 启动脚本文件
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        # 4. 安装 config 配置参数 yaml 文件
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eddyzhou, aryanafrouzi',
    maintainer_email='e23zhou@watonomous.ca, aafrouzi@watonomous.ca',
    description='生产者示例包：定时生成并发布坐标数据',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        # 定义终端可执行程序脚本入口
        'console_scripts': [
            'producer_node = producer.producer_node:main'
        ],
    },
)

