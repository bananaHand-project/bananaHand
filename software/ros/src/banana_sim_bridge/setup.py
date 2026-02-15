from glob import glob
import os

from setuptools import find_packages, setup

package_name = "banana_sim_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        ("share/" + package_name, ["requirements.txt"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="dew.bhaumik8@gmail.com",
    description="MuJoCo simulation bridge scaffold for BananaHand.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "sim_bridge_node = banana_sim_bridge.sim_bridge_node:main",
            "rollout_policy = banana_sim_bridge.rollout:main",
        ],
    },
)
