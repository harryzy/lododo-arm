from setuptools import find_packages, setup
from glob import glob

package_name = "arm_description"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # launch setup
        ("share/" + package_name + "/launch", glob("launch/py/*_launch.py")),
        # urdf setup - only install arm.xacro (exclude lekiwi backup files)
        ("share/" + package_name + "/urdf", ["urdf/arm.xacro"]),
        ("share/" + package_name + "/urdf", ["urdf/urdf.rviz"]),
        # config setup
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        # meshes setup
        ("share/" + package_name + "/urdf/meshes", glob("urdf/meshes/*.stl")),
        ("share/" + package_name + "/urdf/meshes", glob("urdf/meshes/*.dae")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lododo",
    maintainer_email="contect@lododo.org",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    # entry_points={
    #     'console_scripts': [
    #         'arm_description = arm_description.arm_description:main'
    #     ],
    # },
)
