from glob import glob
import os

from setuptools import find_packages, setup

package_name = "banana_hand_fsr_visualizer"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools", "PySide6"],
    zip_safe=True,
    maintainer="lokesh",
    maintainer_email="lokeshpatel257@gmail.com",
    description="2D ROS2 fingertip force visualizer for the Banana Hand.",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "fsr_visualizer = banana_hand_fsr_visualizer.fsr_visualizer_node:main",
            "fsr_test_publisher = banana_hand_fsr_visualizer.fsr_test_publisher:main",
        ],
    },
)
