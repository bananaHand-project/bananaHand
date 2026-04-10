from glob import glob
import os

from setuptools import find_packages, setup

package_name = "banana_hand_visualization"


def package_files(pattern: str) -> list[tuple[str, list[str]]]:
    files = glob(pattern, recursive=True)
    data_files: list[tuple[str, list[str]]] = []
    for file_path in files:
        if os.path.isdir(file_path):
            continue
        rel_path = os.path.relpath(file_path, ".")
        install_dir = os.path.join("share", package_name, os.path.dirname(rel_path))
        data_files.append((install_dir, [file_path]))
    return data_files


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ]
    + package_files("mujoco/**/*"),
    install_requires=["setuptools", "mujoco", "PySide6"],
    zip_safe=True,
    maintainer="lokesh",
    maintainer_email="lokeshpatel257@gmail.com",
    description="MuJoCo-backed Banana Hand visualization for ROS 2 and Foxglove.",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "mujoco_visualizer = banana_hand_visualization.mujoco_visualizer_node:main",
            "standalone_viewer = banana_hand_visualization.standalone_viewer:main",
            "fsr_visualizer = banana_hand_visualization.fsr_visualizer_node:main",
            "fsr_test_publisher = banana_hand_visualization.test_force_publisher:main",
            "fsr_visualizer_test_publisher = banana_hand_visualization.test_force_publisher:main",
            "test_actuator_publisher = banana_hand_visualization.test_actuator_publisher:main",
            "test_force_publisher = banana_hand_visualization.test_force_publisher:main",
        ],
    },
)
