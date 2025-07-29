from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sheep_follower'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis Prieto López',
    maintainer_email='lpril@unileon.es',
    description='ROS2 package for autonomous sheep following using YOLO '
                'detection',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_sheep_follower = sheep_follower.nav2_sheep_follower:main',
            'direct_sheep_follower = sheep_follower.direct_sheep_follower:main',
            'wolf_distance_detector = sheep_follower.wolf_distance_detector:main',
        ],
    },
)
