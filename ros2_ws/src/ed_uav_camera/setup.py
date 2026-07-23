"""Install the isolated ED UAV UVC transport package."""

from setuptools import find_packages, setup


package_name = "ed_uav_camera"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/dual_uvc.launch.py"]),
        (
            f"share/{package_name}/config",
            ["config/camera_profiles.yaml", "config/fake_dual_camera_plan.json"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ED UAV maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Independent USB2 UVC transport launch and validation contracts.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "fake_camera_surface = ed_uav_camera.fake_cli:main",
            "fake_image_device = ed_uav_camera.fake_image_device:main",
        ]
    },
)
