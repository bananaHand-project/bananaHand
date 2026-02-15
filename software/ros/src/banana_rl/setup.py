from glob import glob
import os

from setuptools import find_packages, setup

package_name = "banana_rl"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        ("share/" + package_name, ["requirements.txt"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="dew.bhaumik8@gmail.com",
    description="RL training and evaluation scaffold for MuJoCo dexterous-hand tasks.",
    license="TODO",
    entry_points={
        "console_scripts": [
            "train_rl = banana_rl.train:main",
            "eval_rl = banana_rl.eval:main",
        ],
    },
)
