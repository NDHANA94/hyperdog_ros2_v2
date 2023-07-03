from setuptools import setup
import os
from glob import glob

package_name = 'hyperdog_ros2_pybullet_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nipun',
    maintainer_email='nipun.dhananjaya@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hyperdog_pybullet_node = hyperdog_ros2_pybullet_sim.hyperdog_pybullet_node:main'
        ],
    },
)
