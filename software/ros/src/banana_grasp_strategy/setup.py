from glob import glob
import os

from setuptools import find_packages, setup

package_name = "banana_grasp_strategy"

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
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="dew.bhaumik8@gmail.com",
    description="Grasp strategy prediction package scaffold for BananaHand.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "grasp_strategy_node = banana_grasp_strategy.grasp_strategy_node:main",
        ],
    },
)
