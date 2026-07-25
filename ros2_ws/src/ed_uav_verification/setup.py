from glob import glob

from setuptools import find_packages, setup


PACKAGE_NAME = "ed_uav_verification"


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="ED UAV maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Deterministic offline fakes and fault injection for ED UAV ROS integration.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ed-uav-fake-fcu = ed_uav_verification.fake_fcu:main",
            "ed-uav-verify = ed_uav_verification.cli:main",
            "ed-uav-verify-ros = ed_uav_verification.ros_node:main",
        ],
    },
)
