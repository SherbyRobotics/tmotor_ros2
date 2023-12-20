from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tmotor_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='clocal',
    maintainer_email='clocal@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "tmotor_ros2 = tmotor_ros2.tmotor_ros2:main",
            "basic_2dof_controller = tmotor_ros2.basic_robot_controller:main",
            "pyro_2dof_controller = tmotor_ros2.pyro_robot_controller:main"        
        ],
    },
)
