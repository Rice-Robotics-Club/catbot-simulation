from setuptools import find_packages, setup
import os
from glob import glob

package_name = "catbot_sim"
yaml_launch_files = glob(os.path.join("launch", "*.launch.yml"))
py_launch_files = glob(os.path.join("launch", "*.launch.py"))
mesh_files = glob(os.path.join("meshes", "*"))
world_files = glob(os.path.join("worlds", "*"))
config_files = glob(os.path.join("config", "*"))

executables = glob(os.path.join(package_name, "*_node.py"))


def console_script(filename: str) -> str:
    name = filename[len(package_name)+1:len(filename)-3]
    return f"{name} = {package_name}.{name}:main"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", yaml_launch_files),
        ("share/" + package_name + "/launch", py_launch_files),
        ("share/" + package_name + "/meshes", mesh_files),
        ("share/" + package_name + "/worlds", world_files),
        ("share/" + package_name + "/config", config_files),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nico L",
    maintainer_email="nicholasl23638@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": list(map(console_script, executables)),
    },
)
