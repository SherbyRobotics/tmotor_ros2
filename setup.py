from setuptools import setup
import os
from glob import glob

package_name = 'tmotor_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mecanique',
    maintainer_email='mecanique@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "tmotor_ros2 = tmotor_ros2.tmotor_ros2:main",
            "basic_2dof_controller = tmotor_ros2.basic_2dof_controller:main",
            "pyro_2dof_controller = tmotor_ros2.pyro_2dof_controller:main"
        ],
    },
)
