from setuptools import find_packages, setup
from glob import glob

package_name = 'arm_voice_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # launch setup
    ("share/" + package_name + "/launch", glob("launch/*_launch.py")),
    (f'share/{package_name}/config', ['config/vosk_config.yaml']),
    (f'share/{package_name}/config', glob("config/*.rviz")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lododo',
    maintainer_email='contect@lododo.org',
    description='Voice interface using Vosk to publish arm commands',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_voice_node = arm_voice_interface.arm_voice_node:main',
        ],
    },
)
