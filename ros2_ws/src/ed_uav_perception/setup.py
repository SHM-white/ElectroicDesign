"""Install the ED UAV perception detector runtime package."""

from glob import glob

from setuptools import find_packages, setup


package_name = "ed_uav_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy", "opencv-python-headless>=4.5"],
    zip_safe=True,
    maintainer="ED UAV maintainers",
    maintainer_email="maintainers@example.invalid",
    description="Dual-camera AprilTag detection, fusion, and prescribed target pose runtime.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detector_node = ed_uav_perception.detector_node:main",
            "target_observation_node = ed_uav_perception.target_observation_node:main",
            "visual_servo_node = ed_uav_perception.visual_servo_node:main",
            "narrow_detector = ed_uav_perception.narrow_detector_node:main_narrow",
            "wide_detector = ed_uav_perception.wide_detector_node:main_wide",
            "target_fusion = ed_uav_perception.target_fusion_node:main",
            "perception_visualizer = ed_uav_perception.perception_visualizer_node:main",
        ],
    },
)
