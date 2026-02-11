from setuptools import find_packages, setup

package_name = "banana_hand_tracking"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/webcam.launch.py",
                "launch/hand_tracking.launch.py",
                "launch/vision_teleop.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools", "mediapipe"],
    zip_safe=True,
    maintainer="TODO",
    maintainer_email="dew.bhaumik8@gmail.com",
    description="Webcam hand tracking package (skeleton).",
    license="TODO",
    entry_points={
        "console_scripts": [
            "webcam_node = banana_hand_tracking.webcam_node:main",
            "hand_tracking_node = banana_hand_tracking.hand_tracking_node:main",
        ],
    },
)
