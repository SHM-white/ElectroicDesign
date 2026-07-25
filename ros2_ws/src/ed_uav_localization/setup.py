from setuptools import find_packages, setup
from glob import glob
import os


package_name = "ed_uav_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config", "fields"), glob("config/fields/*.yaml")),
    ],
    install_requires=["PyYAML>=6", "pydantic>=2,<3"],
    zip_safe=True,
    maintainer="ED UAV maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Strict field profiles and coordinate anchoring for ED UAV localization.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "source_supervisor = ed_uav_localization.source_supervisor:main",
            "field_anchor = ed_uav_localization.field_anchor:main",
        ],
    },
)
