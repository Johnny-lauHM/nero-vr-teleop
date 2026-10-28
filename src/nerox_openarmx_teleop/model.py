from __future__ import annotations

from collections.abc import Mapping, Sequence

import numpy as np


HARDWARE_JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 8))
OPENARMX_LEFT_JOINT_NAMES = tuple(
    f"openarmx_left_joint{index}" for index in range(1, 8)
)
OPENARMX_RIGHT_JOINT_NAMES = tuple(
    f"openarmx_right_joint{index}" for index in range(1, 8)
)
OPENARMX_JOINT_NAMES = OPENARMX_LEFT_JOINT_NAMES + OPENARMX_RIGHT_JOINT_NAMES

# OpenArmX left/right map to the operator's left/right controllers.  On this
# NERO assembly those are physical arm_b/arm_a respectively.
OPENARMX_SIDE_TO_HARDWARE_ARM = {"left": "arm_b", "right": "arm_a"}


def _joint_vector(values: Sequence[float], label: str) -> np.ndarray:
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (7,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must contain seven finite joint values")
    return result


def hardware_to_model(
    hardware: Mapping[str, Sequence[float]],
) -> np.ndarray:
    """Return OpenArmX [left(arm_b), right(arm_a)] model coordinates."""

    # The accepted assembly URDF bakes the physical zero convention into its
    # joint origins. OpenArmX therefore consumes the live encoder angles
    # unchanged; only the left/right branch ordering differs.
    arm_b = _joint_vector(hardware["arm_b"], "arm_b")
    arm_a = _joint_vector(hardware["arm_a"], "arm_a")
    return np.concatenate([arm_b, arm_a])


def model_to_hardware(model_positions: Sequence[float]) -> dict[str, np.ndarray]:
    """Convert an OpenArmX 14-joint result to physical NERO coordinates."""

    values = np.asarray(model_positions, dtype=np.float64)
    if values.shape != (14,) or not np.all(np.isfinite(values)):
        raise ValueError("OpenArmX target must contain fourteen finite joint values")
    return {"arm_a": values[7:].copy(), "arm_b": values[:7].copy()}
