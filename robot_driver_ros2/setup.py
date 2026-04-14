import os
from glob import glob

from setuptools import find_packages, setup

package_name = "robot_driver_ros2"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="User",
    maintainer_email="user@example.com",
    description="ROS2 Foxy package: ROS1 robot_driver logic with rclpy (DAS + UVC).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_view_single = robot_driver_ros2.camera_view_single:main",
            "databus_single = robot_driver_ros2.databus_single:main",
            "tactile_dual_print = robot_driver_ros2.tactile_dual_print:main",
            "left_das_controller_infer = robot_driver_ros2.left_das_controller_infer:main",
            "right_das_controller_infer = robot_driver_ros2.right_das_controller_infer:main",
        ],
    },
)
