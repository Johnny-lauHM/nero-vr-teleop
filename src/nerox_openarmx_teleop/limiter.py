from __future__ import annotations

from collections.abc import Sequence

import numpy as np


class OpenArmXJointStepLimiter:
    """The per-joint step limiter from the official OpenArmX VR node."""

    SLOW_STEP_DEG = (4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0)
    FAST_STEP_DEG = (20.0, 16.0, 12.0, 12.0, 14.0, 8.0, 12.0)
    ENABLE_THRESHOLD_DEG = (20.0, 16.0, 12.0, 12.0, 14.0, 8.0, 12.0)

    def __init__(self) -> None:
        self.slow_step = np.radians(self.SLOW_STEP_DEG)
        self.fast_step = np.radians(self.FAST_STEP_DEG)
        self.enable_threshold = np.radians(self.ENABLE_THRESHOLD_DEG)

    @staticmethod
    def _vector(values: Sequence[float], label: str) -> np.ndarray:
        result = np.asarray(values, dtype=np.float64)
        if result.shape != (7,) or not np.all(np.isfinite(result)):
            raise ValueError(f"{label} must contain seven finite values")
        return result

    def limit(
        self,
        target: Sequence[float],
        measured: Sequence[float],
        *,
        fast_mode: bool = False,
    ) -> np.ndarray:
        desired = self._vector(target, "OpenArmX target")
        feedback = self._vector(measured, "NERO feedback")
        max_step = self.fast_step if fast_mode else self.slow_step
        delta = desired - feedback
        apply_limit = np.abs(delta) > self.enable_threshold
        limited_delta = np.clip(delta, -max_step, max_step)
        return feedback + np.where(apply_limit, limited_delta, delta)
