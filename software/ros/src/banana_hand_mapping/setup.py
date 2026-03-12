from glob import glob
import os

from setuptools import find_packages, setup

package_name = "banana_hand_mapping"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lokesh",
    maintainer_email="lokeshpatel257@gmail.com",
    description="Hand teleop to motor command mapping.",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            (
                "teleop_to_tx_positions = "
                "banana_hand_mapping.teleop_to_tx_positions_node:main"
            ),
            "manual_control = banana_hand_mapping.manual_control_node:main",
        ],
    },
)
