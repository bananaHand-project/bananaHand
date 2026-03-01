from glob import glob
import os

from setuptools import find_packages, setup

package_name = "banana_sfm_reconstruction"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyrealsense2", "opencv-python", "numpy"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="dew.bhaumik8@gmail.com",
    description="SfM and 3D reconstruction package scaffold for BananaHand.",
    license="TODO",
    entry_points={
        "console_scripts": [
            (
                "reconstruction_node = "
                "banana_sfm_reconstruction.reconstruction_node:main"
            ),
        ],
    },
)
