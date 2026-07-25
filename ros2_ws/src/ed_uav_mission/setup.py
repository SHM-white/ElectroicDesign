from setuptools import find_packages, setup
from glob import glob
import os


package_name = "ed_uav_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config", "missions"), glob("config/missions/*.yaml")),
    ],
    install_requires=["setuptools", "PyYAML>=6"],
    zip_safe=True,
    maintainer="ED UAV maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Plugin-based mission orchestration for ED UAV.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_executor = ed_uav_mission.executor:main",
        ],
    },
)
