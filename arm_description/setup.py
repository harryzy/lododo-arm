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
        # urdf setup
        ("share/" + package_name + "/urdf", glob("urdf/*.xacro")),
        ("share/" + package_name + "/urdf", glob("urdf/*.urdf")),
        ("share/" + package_name + "/urdf", glob("urdf/*.rviz")),
        # config setup
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        # meshes setup
        ("share/" + package_name + "/meshes", glob("meshes/*.stl")),
        ("share/" + package_name + "/meshes", glob("meshes/*.dae")),
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
