"""Install the ED UAV perception detector runtime package."""

from setuptools import find_packages, setup


package_name = "ed_uav_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "opencv-python-headless>=4.5"],
    zip_safe=True,
    maintainer="ED UAV maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Narrow-camera detector runtime for ED UAV perception.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detector_node = ed_uav_perception.detector_node:main",
        ],
    },
)
