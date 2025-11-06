from setuptools import setup
from glob import glob

package_name = "arm_perception_yolo"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        # Ament index
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # Install package.xml
        ("share/" + package_name, ["package.xml"]),
        # Configuration and launch files
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*launch.py")),
        # Install scripts to share/arm_perception_yolo/scripts
        ("share/" + package_name + "/scripts", ["scripts/run_yolo_venv.sh"]),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=True,
    maintainer="lododo",
    maintainer_email="contect@lododo.org",
    description="YOLO perception",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "yolo_detector = arm_perception_yolo.yolo_detector:main",
            "projection_node = arm_perception_yolo.projection_node:main",
            "triangulation_node = arm_perception_yolo.triangulation_node:main",
        ],
    },
)
