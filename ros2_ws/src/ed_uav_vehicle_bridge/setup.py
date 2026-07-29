from glob import glob

from setuptools import find_packages, setup

package_name = "ed_uav_vehicle_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "PROTOCOL.md"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ED UAV maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Authenticated bounded UDP v1 vehicle and HMI bridge.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vehicle_bridge = ed_uav_vehicle_bridge.entrypoint:main",
            "fake_vehicle_source = ed_uav_vehicle_bridge.fake_source:main",
        ],
    },
)
