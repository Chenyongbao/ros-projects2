from setuptools import setup


package_name = 'neobot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mrchen',
    maintainer_email='mrchen@example.com',
    description='Custom control and autonomy nodes for the Neobot simulation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_monitor = neobot_control.odom_monitor:main',
            'square_driver = neobot_control.square_driver:main',
            'point_controller = neobot_control.point_controller:main',
            'obstacle_stop = neobot_control.obstacle_stop:main',
            'navigate_to_pose = neobot_control.navigate_to_pose:main',
            'path_follower = neobot_control.path_follower:main',
            'path_demo = neobot_control.path_demo:main',
        ],
    },
)
