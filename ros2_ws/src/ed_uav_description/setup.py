from glob import glob

from setuptools import find_packages, setup


package_name = "ed_uav_description"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/urdf", glob("urdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ED UAV maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Static ED UAV sensor-frame model and calibration boundary.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "validate_calibration = ed_uav_description.calibration:main",
        ],
    },
)
