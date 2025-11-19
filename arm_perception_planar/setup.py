from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_perception_planar'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hurry',
    maintainer_email='jeladeer@msn.com',
    description='Planar perception system for cube detection and localization using geometric analysis',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cube_detector_node = arm_perception_planar.cube_detector_node:main',
            'planar_localization_node = arm_perception_planar.planar_localization_node:main',
        ],
    },
)
