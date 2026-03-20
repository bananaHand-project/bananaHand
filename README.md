# BananaHand

**BananaHand** is an open-source 16 DOF/6 DOA humanoid hand designed for physical AI research and deployment on light-weight robot arms.

*insert image of hand*

BananaHand features human-like proportions with no fore-arm, 4lb payload, integrated palm and finger tip FSR based force sensing, and a low BOM cost of 1200$CAD.

With the project being open-source as a top priority, all resources used in the creation are freely available and open-source themselves (KiCad, OnShape, Foxglove, Embassy-rs, etc.).

*We're in continuous development and actively seeking contributors, research partners, and company collaborators to shape the next generation of practical humanoid systems. Ready to join the future of open-source robotics?*

## Repo Structure

- `docs/` contains build instructions, getting started guide, and more.
- `firmware/` contains the code that runs on the bananaHand main board.
  - `firmware/primary-g474/` contains the firmware run on the primary MCU (stm32g474).
  - `firmware/sensing-c071/` contains the firmware run on the sensing MCU (stm32c071).
  - firmware/experiments/` contains code used in the development of the hand.
- `hardware/` contains both electrical and mechanical CAD models, BOMs, and detailed descriptions.
- `software/` contains ROS and Foxglove implementations.

---
***We can't wait to see what you grasp!***

![bananaHand grasping objects](/assets/images/bh_grasping_objects.jpg)

Made with ❤️ in 🇨🇦 at 🪿.
