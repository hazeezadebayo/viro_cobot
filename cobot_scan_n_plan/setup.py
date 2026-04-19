import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'cobot_scan_n_plan'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='azeez adebayo',
    maintainer_email='hazeezadebayo@gmail.com',
    description='Surgical ROS 2 migration of the Scan-and-Plan workflow',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rs_point_cloud = scripts.rs_point_cloud:main',
            'surface_scan = scripts.surface_scan_node:main',
            'surface_plan_node = scripts.surface_plan_node:main',
            'surface_traj_node = scripts.surface_traj_node:main',
        ],
    },
)
