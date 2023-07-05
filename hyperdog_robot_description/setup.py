from setuptools import setup
import os
from glob import glob

package_name = 'hyperdog_robot_description'

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
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes/hyperdog_v1'), glob('meshes/hyperdog_v1/*.stl')),
        (os.path.join('share', package_name, 'meshes/hyperdog_v2'), glob('meshes/hyperdog_v2/*.stl'))
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
        ],
    },
)
