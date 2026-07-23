from setuptools import find_packages, setup


package_name = "ed_uav_localization"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["PyYAML>=6", "pydantic>=2,<3"],
    zip_safe=True,
    maintainer="ED UAV maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Strict field profiles and coordinate anchoring for ED UAV localization.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
