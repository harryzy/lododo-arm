from setuptools import find_packages, setup

package_name = 'arm_planning_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lododo',
    maintainer_email='contect@lododo.org',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_planning_py_node = arm_planning_py.arm_planning_py_node:main',
            'test_planning_py_node = arm_planning_py.test.test_planning_py_node:main',
            'yolo_detection_node = arm_planning_py.yolo_detection_node:main',
            'arm_command_interface = arm_planning_py.arm_command_interface:main',
            'test_hand_delivery = arm_planning_py.test_hand_delivery:main',
            'test_hand_delivery_integration = arm_planning_py.test_hand_delivery_integration:main',
        ],
    },
)
