from glob import glob
from os import path

from setuptools import find_packages, setup


package_name = "ed_uav_gazebo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (path.join("share", package_name, "launch"), glob("launch/*.py")),
        (path.join("share", package_name, "config"), glob("config/*.yaml")),
        (path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
        (path.join("share", package_name, "models", "ed_quadrotor"), glob("models/ed_quadrotor/*")),
        (path.join("share", package_name, "models", "apriltag_marker"), ["models/apriltag_marker/model.config", "models/apriltag_marker/model.sdf"]),
        (path.join("share", package_name, "models", "apriltag_marker", "materials", "textures"), glob("models/apriltag_marker/materials/textures/*")),
        (path.join("share", package_name, "models", "d_task_car"), glob("models/d_task_car/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ED UAV maintainers",
    maintainer_email="maintainers@example.com",
    description="Gazebo Fortress simulator arena and adapters for ED UAV",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_fcu = ed_uav_gazebo.sim_fcu:main",
            "sim_localization = ed_uav_gazebo.sim_localization:main",
            "sim_vehicle_telemetry = ed_uav_gazebo.sim_vehicle_telemetry:main",
            "gazebo_pointcloud_normalizer = ed_uav_gazebo.gazebo_pointcloud_normalizer:main",
            "planar_odom_fuser = ed_uav_gazebo.planar_odom_fuser:main",
            "sim_car_controller = ed_uav_gazebo.sim_car_controller:main",
            "sim_mission_starter = ed_uav_gazebo.sim_mission_starter:main",
            "camera_debug = ed_uav_gazebo.camera_debug:main",
        ],
    },
)
