from glob import glob
from os import path

from setuptools import find_packages, setup


package_name = "ed_uav_gazebo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (path.join("share", package_name, "launch"), glob("launch/*.py")),
        (path.join("share", package_name, "config"), glob("config/*.yaml")),
        (path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
        (path.join("share", package_name, "models", "ed_quadrotor"), glob("models/ed_quadrotor/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ED UAV maintainers",
    maintainer_email="maintainers@example.com",
    description="Gazebo Fortress simulator arena and adapters for ED UAV",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_fcu = ed_uav_gazebo.sim_fcu:main",
            "sim_localization = ed_uav_gazebo.sim_localization:main",
            "gazebo_pointcloud_normalizer = ed_uav_gazebo.gazebo_pointcloud_normalizer:main",
        ],
    },
)
