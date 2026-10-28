# Third-party notices

| Component | Location | Source | License | Changes |
| --- | --- | --- | --- | --- |
| openarmx_teleop_bridge_vr | `vendor/openarmx_teleop_bridge_vr/` | [openarmx/openarmx_teleop_vr](https://github.com/openarmx/openarmx_teleop_vr) @ `a3da741` | CC BY-NC-SA 4.0, © 2026 Chengdu Changshu Robot Co., Ltd. | Datagram parse errors no longer print the raw buffer; periodic raw-pose debug log removed. |
| openarmx-arm-driver 1.3.3 (PinocchioTeleopCore) | installed into `vendor/openarmx_python/` by `scripts/install_python_deps.sh`; not redistributed | [PyPI](https://pypi.org/project/openarmx-arm-driver/) | CC BY-NC-SA 4.0, © 2025 Wei Lindong | None (installed as published). |
| OpenArmX Quest APK | downloaded by `scripts/install_quest_apk.sh`; not redistributed | [openarmx/openarmx_teleop_vr_apk](https://github.com/openarmx/openarmx_teleop_vr_apk) @ `27808af` | CC BY-NC-SA 4.0 | None (SHA256-pinned). |
| agx_arm_ctrl, agx_arm_msgs | `vendor/agx_arm_ros/src/` | [agilexrobotics/agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros) | Apache-2.0 | Snapshot of the driver version validated with this stack; differs from the pinned submodule (`fw_version`/`revo2_type` parameters, `speed_percent` default, `close()` that drops the command gate, Revo2 Pro/Touch effectors). |
| agx_arm_ros (agx_arm_description) | `third_party/agx_arm_ros` (submodule) | [agilexrobotics/agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros) @ `c73d33f` | Apache-2.0 | None. |
| pyAgxArm | `vendor/pyAgxArm/` | [agilexrobotics/pyAgxArm](https://github.com/agilexrobotics/pyAgxArm) @ `8cd90f9`, © Agilex Robotics | LGPL-3.0 (see `vendor/pyAgxArm/LICENSE`) | Modified: CAN communication and NERO default/v112 drivers updated for the dual-NERO setup; tests, docs and assets omitted. |
| XRoboToolkit PC Service / Pybind | not included; install separately | [XR-Robotics](https://github.com/XR-Robotics) | see upstream | — |

The teleoperation node in `src/nerox_openarmx_teleop/` builds on the OpenArmX
VR teleoperation design and PinocchioTeleopCore; the repository as a whole is
therefore distributed under CC BY-NC-SA 4.0 (non-commercial use only).
Commercial use requires separate licensing from the OpenArmX authors.
