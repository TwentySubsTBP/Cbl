from setuptools import setup
import os
from glob import glob

package_name = 'my_tb3_world'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*.py')),
        (os.path.join('share',package_name,'worlds'),glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='test',
    maintainer_email='test@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mover = my_tb3_world.mover:main',
            'go_to_goal = my_tb3_world.go_to_goal:main',
            'obstacle_info = my_tb3_world.obstacle_info:main',
            'obstacle_info_sub = my_tb3_world.obstacle_info_subscriber:main',
            'ph_sensor = my_tb3_world.ph_sensor:main',
            'dt_supervisor = my_tb3_world.dt_supervisor:main',
            'anomaly_zone_viz = my_tb3_world.anomaly_zone_viz:main',
            'hazard_manager = my_tb3_world.hazard_manager:main',
            'comms_watchdog = my_tb3_world.comms_watchdog:main',
            'cmd_mux = my_tb3_world.cmd_mux:main',
            'anomaly_alert = my_tb3_world.anomaly_alert:main',
            'latency_logger = my_tb3_world.latency_logger:main',
        ],
    },
)
