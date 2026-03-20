#!/usr/bin/env python3
"""Launch the Banana Hand MuJoCo viewer without ROS bringup."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import mujoco
import mujoco.viewer


def main() -> None:
    model_path = Path(get_package_share_directory("banana_hand_visualization")) / "mujoco" / "scene.xml"
    model = mujoco.MjModel.from_xml_path(str(model_path))
    mujoco.viewer.launch(model)


if __name__ == "__main__":
    main()
