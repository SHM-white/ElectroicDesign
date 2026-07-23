"""Setuptools metadata for the optional ED UAV lidar package."""

from glob import glob

from setuptools import setup


PACKAGE_NAME = "ed_uav_lidar"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
        (f"share/{PACKAGE_NAME}/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ED UAV maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Optional lossless Mid-360 and generic PointCloud2 transport contract.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "generic_monitor = ed_uav_lidar.generic_monitor:main",
            "mid360_adapter = ed_uav_lidar.mid360_adapter:main",
            "lidar_replay = ed_uav_lidar.replay:main",
        ],
    },
)
