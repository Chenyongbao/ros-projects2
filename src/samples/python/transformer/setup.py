# setup.py
# ROS 2 Python 包 transformer 的打包与安装配置文件

import os
from glob import glob
from setuptools import setup

package_name = 'transformer'

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
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch',
                                                                          '*.launch.py'))),
        # 4. 安装 config 配置参数 yaml
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eddyzhou, aryanafrouzi',
    maintainer_email='e23zhou@watonomous.ca, aafrouzi@watonomous.ca',
    description='转换器示例包：解析原始未滤波数据并批量重构为滤波数据',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        # 声明终端可执行节点入口
        'console_scripts': [
            'transformer_node = transformer.transformer_node:main'
        ],
    },
)

