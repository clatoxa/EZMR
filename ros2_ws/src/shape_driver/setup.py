import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'shape_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launchfiles und Parameterdateien installieren
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='leo.kilian@stud.htwk-leipzig.de',
    description='Zeitgesteuerte Fahrfolgen über geometry_msgs/Twist für turtlesim und den ESP32-Roboter',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'shape_driver_node = shape_driver.shape_driver_node:main',
        'calib_drive = shape_driver.calib_drive:main',
        'obstacle_stop = shape_driver.obstacle_stop:main',
        'distance_warner = shape_driver.distance_warner:main',
	'wander = shape_driver.wander:main',
	'odom = shape_driver.odom:main',
	'sonar_map = shape_driver.sonar_map:main',
        ],
    },
)
