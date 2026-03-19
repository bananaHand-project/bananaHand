from glob import glob
import os

from setuptools import find_packages, setup

package_name = "banana_grasp_classification"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools", "numpy", "open3d"],
    zip_safe=True,
    maintainer="dbhaumik",
    maintainer_email="dew.bhaumik8@gmail.com",
    description="Point-cloud preprocessing utilities for grasp classification.",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            (
                "ground_plane_removal_node = "
                "banana_grasp_classification.ground_plane_removal_node:main"
            ),
            (
                "grasp_rule_classifier_node = "
                "banana_grasp_classification.grasp_rule_classifier_node:main"
            ),
            (
                "scan_grasp_pipeline_node = "
                "banana_grasp_classification.scan_grasp_pipeline_node:main"
            ),
        ],
    },
)
