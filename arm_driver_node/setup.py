from setuptools import find_packages, setup
from glob import glob

package_name = "arm_driver_node"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"],include=[
        'arm_driver_node', 
        'arm_driver_node.*',
        'arm_driver_node.sdk.*', 
        'arm_driver_node.sdk.ftservo.*',
        'arm_driver_node.sdk.ftservo.scservo_sdk.*'
    ]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # launch setup
        ('share/' + package_name + '/launch', glob("launch/*_launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lododo",
    maintainer_email="contect@lododo.org",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm_driver_node = arm_driver_node.arm_driver_node:main",
            "arm_driver_node_sim = arm_driver_node.arm_driver_node_sim:main",
            "arm_driver_node_sdk_test = arm_driver_node.arm_driver_node_sdk_test:main",
        ],
    },
)
