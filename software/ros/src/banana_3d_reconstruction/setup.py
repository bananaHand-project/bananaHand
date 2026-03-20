from glob import glob
import os

from setuptools import find_packages, setup

package_name = "banana_3d_reconstruction"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "opencv-python",
        "open3d",
        "pyrealsense2",
    ],
    zip_safe=True,
    maintainer="dbhaumik",
    maintainer_email="dew.bhaumik8@gmail.com",
    description="RGB-D tabletop object scanning and object-only point cloud reconstruction.",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "object_scan_node = banana_3d_reconstruction.object_scan_node:main",
        ],
    },
)
