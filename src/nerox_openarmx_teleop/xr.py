from __future__ import annotations

import math
from typing import Any

import numpy as np

# These are the matrices used by the official OpenArmX VR node. The Quest
# application publishes AIM poses in gripper mode and GRIP poses in hand mode.
QUEST_GRIPPER_AXIS = np.eye(3, dtype=np.float64)
QUEST_HAND_AXIS = np.asarray(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)

# Validated NERO/XR TCP basis for this assembly.  Keep translation and rotation
# names separate even though the current calibration uses the same matrix for
# both, so either convention can be adjusted independently after commissioning.
NERO_TCP_TRANSLATION_AXIS = np.asarray(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)
NERO_TCP_ORIENTATION_AXIS = NERO_TCP_TRANSLATION_AXIS.copy()


def yaw_adjusted_axis(axis_matrix: Any, yaw_deg: float) -> np.ndarray:
    """Rotate an XR-to-robot basis about the robot world's vertical axis."""

    axis = np.asarray(axis_matrix, dtype=np.float64).reshape(3, 3)
    yaw = float(yaw_deg)
    if not np.all(np.isfinite(axis)):
        raise ValueError("axis matrix must be finite")
    if not math.isfinite(yaw):
        raise ValueError("input yaw must be finite")
    angle = math.radians(yaw)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    rotation = np.asarray(
        [
            [cosine, -sine, 0.0],
            [sine, cosine, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    adjusted = rotation @ axis
    adjusted[np.abs(adjusted) < 1e-15] = 0.0
    return adjusted


def bounded_unit(value: Any, label: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} is not finite")
    return min(1.0, max(0.0, result))


def validated_pose(value: Any, label: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (7,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must be [x,y,z,qx,qy,qz,qw]")
    norm = float(np.linalg.norm(result[3:]))
    if not 0.5 <= norm <= 1.5:
        raise ValueError(f"{label} quaternion norm is invalid: {norm:.6f}")
    result = result.copy()
    result[3:] /= norm
    return result


def _quaternion_to_rotation(pose: np.ndarray) -> np.ndarray:
    x, y, z, w = pose[3:]
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _rotation_to_quaternion(matrix: np.ndarray) -> tuple[float, float, float, float]:
    rotation = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        values = (
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        )
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(
                1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]
            ) * 2.0
            values = (
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
            )
        elif index == 1:
            scale = math.sqrt(
                1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]
            ) * 2.0
            values = (
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
            )
        else:
            scale = math.sqrt(
                1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]
            ) * 2.0
            values = (
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            )
    quaternion = np.asarray(values, dtype=np.float64)
    quaternion /= float(np.linalg.norm(quaternion))
    return tuple(float(value) for value in quaternion)


class RelativePoseTracker:
    """Clutch-anchored Quest pose with independent position/rotation bases."""

    def __init__(
        self,
        pose_type: Any,
        axis_matrix: np.ndarray,
        orientation_matrix: np.ndarray | None = None,
    ) -> None:
        self._pose_type = pose_type
        self._position_axis = np.asarray(axis_matrix, dtype=np.float64).reshape(3, 3)
        self._orientation_axis = np.asarray(
            axis_matrix if orientation_matrix is None else orientation_matrix,
            dtype=np.float64,
        ).reshape(3, 3)
        self.reset()

    def reset(self) -> None:
        self._reference: np.ndarray | None = None
        self._robot_reference_rotation: np.ndarray | None = None
        self._current: np.ndarray | None = None
        self._input_world_position_delta: np.ndarray | None = None
        self._mapped_world_position: np.ndarray | None = None
        self._enabled = False

    def position_diagnostics(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None:
        """Return copies of reference, current, raw delta, and mapped delta."""

        if (
            self._reference is None
            or self._current is None
            or self._input_world_position_delta is None
            or self._mapped_world_position is None
        ):
            return None
        return (
            self._reference.copy(),
            self._current.copy(),
            self._input_world_position_delta.copy(),
            self._mapped_world_position.copy(),
        )

    def update(
        self,
        raw_pose: np.ndarray,
        enabled: bool,
        timestamp_s: float,
        robot_reference_rotation: np.ndarray | None = None,
    ) -> Any | None:
        if not enabled:
            self.reset()
            return None
        current = validated_pose(raw_pose, "relative Quest controller pose")
        if not self._enabled or self._reference is None:
            robot_rotation = np.asarray(
                np.eye(3) if robot_reference_rotation is None else robot_reference_rotation,
                dtype=np.float64,
            )
            if robot_rotation.shape != (3, 3) or not np.all(np.isfinite(robot_rotation)):
                raise ValueError("robot reference rotation must be a finite 3x3 matrix")
            self._reference = current.copy()
            self._robot_reference_rotation = robot_rotation.copy()
            self._enabled = True
            input_world_position_delta = np.zeros(3, dtype=np.float64)
            input_world_rotation_delta = np.eye(3, dtype=np.float64)
        else:
            reference_rotation = _quaternion_to_rotation(self._reference)
            current_rotation = _quaternion_to_rotation(current)
            # This retargeting uses controller-world deltas, not deltas
            # expressed in the controller orientation captured at Grip time.
            input_world_position_delta = current[:3] - self._reference[:3]
            input_world_rotation_delta = current_rotation @ reference_rotation.T

        mapped_world_position = self._position_axis @ input_world_position_delta
        mapped_world_rotation = (
            self._orientation_axis
            @ input_world_rotation_delta
            @ self._orientation_axis.T
        )
        self._current = current.copy()
        self._input_world_position_delta = input_world_position_delta.copy()
        self._mapped_world_position = mapped_world_position.copy()
        robot_rotation = self._robot_reference_rotation
        if robot_rotation is None:
            raise RuntimeError("robot reference rotation was not captured")
        # OpenArmX consumes relative position/rotation in the robot TCP frame
        # and right-multiplies the TCP anchor. Convert the world-frame
        # deltas back into that local convention before calling its IK core.
        mapped_position = robot_rotation.T @ mapped_world_position
        mapped_rotation = robot_rotation.T @ mapped_world_rotation @ robot_rotation
        return self._pose_type(
            position=mapped_position,
            orientation_xyzw=_rotation_to_quaternion(mapped_rotation),
            timestamp=float(timestamp_s),
        )
