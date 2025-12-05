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
         glob(os.path.join('config', '*.yaml')) + glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=[
        'setuptools',
        'numpy',
    ],
    zip_safe=True,
    maintainer='Luis Prieto López',
    maintainer_email='lpril@unileon.es',
    description='ROS2 package for autonomous sheep following using YOLO detection',
    license='MIT',
    keywords=['ROS2', 'robotics', 'YOLO', 'sheep', 'herding', 'navigation', 'Nav2'],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav2_sheep_follower = sheep_follower.nav2_sheep_follower:main',
            'direct_sheep_follower = sheep_follower.direct_sheep_follower:main',
            'wolf_distance_detector = sheep_follower.wolf_distance_detector:main',
        ],
    },
    classifiers=[
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.10',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
    project_urls={
        'Homepage': 'https://github.com/luispri2001/yolo_sheep_follower',
        'Repository': 'https://github.com/luispri2001/yolo_sheep_follower.git',
        'Bug Tracker': 'https://github.com/luispri2001/yolo_sheep_follower/issues',
    },
)
