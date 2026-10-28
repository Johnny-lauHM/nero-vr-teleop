from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from nerox_openarmx_teleop.limiter import OpenArmXJointStepLimiter
from nerox_openarmx_teleop.model import (
    HARDWARE_JOINT_NAMES,
    OPENARMX_JOINT_NAMES,
    OPENARMX_SIDE_TO_HARDWARE_ARM,
    hardware_to_model,
    model_to_hardware,
)
from nerox_openarmx_teleop.xr import (
    NERO_TCP_ORIENTATION_AXIS,
    NERO_TCP_TRANSLATION_AXIS,
    QUEST_HAND_AXIS,
    RelativePoseTracker,
    bounded_unit,
    validated_pose,
    yaw_adjusted_axis,
)


# Keep the IK smoothing inside the official OpenArmX implementation.  The
# packaged solver defaults to four samples [0.4, 0.3, 0.2, 0.1]; this eight
# sample profile reduces frame-to-frame joint changes while adding only about
# 27 ms of weighted delay at the 100 Hz control rate.
OPENARMX_IK_FILTER_WEIGHTS = np.asarray(
    [0.18, 0.17, 0.16, 0.15, 0.13, 0.10, 0.07, 0.04],
    dtype=np.float64,
)
HANDS_UP_J4_RAD = math.pi / 2.0

TACCLAW_STATE_TOPIC = "/nerox/tacclaw/state"
TACCLAW_COMMAND_TOPIC = "/nerox/tacclaw/command"
TACCLAW_APPLIED_TOPICS = {
    "left": "/nerox/tacclaw/applied/left",
    "right": "/nerox/tacclaw/applied/right",
}


class LowElbowPostureGuard:
    """Reject IK solutions outside the configured low-elbow envelope."""

    def __init__(
        self,
        model_path: Path,
        *,
        max_elbow_height_m: float | None,
        max_abs_j2_deg: float | None,
        max_abs_j3_deg: float | None,
        joint_limit_margin_deg: float,
    ) -> None:
        import pinocchio as pin

        self._pin = pin
        self._model = pin.buildModelFromUrdf(str(model_path))
        self._data = self._model.createData()
        self._max_elbow_height_m = max_elbow_height_m
        self._max_abs_j2_rad = (
            None if max_abs_j2_deg is None else math.radians(max_abs_j2_deg)
        )
        self._max_abs_j3_rad = (
            None if max_abs_j3_deg is None else math.radians(max_abs_j3_deg)
        )
        self._joint_limit_margin_rad = math.radians(joint_limit_margin_deg)
        self._side_slices = {"left": slice(0, 7), "right": slice(7, 14)}
        self._elbow_frames = {
            side: self._model.getFrameId(f"openarmx_{side}_link3")
            for side in self._side_slices
        }

    def rejection(self, target_q: Any, active_sides: set[str]) -> str | None:
        q = np.asarray(target_q, dtype=np.float64)
        if q.shape != (14,) or not np.all(np.isfinite(q)):
            return "IK target is not a finite 14-joint vector"

        lower = np.asarray(self._model.lowerPositionLimit, dtype=np.float64)
        upper = np.asarray(self._model.upperPositionLimit, dtype=np.float64)
        margin = self._joint_limit_margin_rad
        for side in sorted(active_sides):
            indices = self._side_slices[side]
            side_q = q[indices]
            # Float32 command recordings can differ from a URDF limit by a few
            # nanoradians; do not turn representation noise into a hold.
            side_lower = lower[indices] + margin - 1e-6
            side_upper = upper[indices] - margin + 1e-6
            outside = np.flatnonzero((side_q < side_lower) | (side_q > side_upper))
            if outside.size:
                joint = int(outside[0]) + 1
                return (
                    f"{side} J{joint} leaves the configured joint-limit margin: "
                    f"q={math.degrees(side_q[joint - 1]):.1f}deg"
                )
            if (
                self._max_abs_j2_rad is not None
                and abs(float(side_q[1])) > self._max_abs_j2_rad
            ):
                return (
                    f"{side} J2 exceeds posture limit: "
                    f"abs(q)={abs(math.degrees(side_q[1])):.1f}deg"
                )
            if (
                self._max_abs_j3_rad is not None
                and abs(float(side_q[2])) > self._max_abs_j3_rad
            ):
                return (
                    f"{side} J3 exceeds posture limit: "
                    f"abs(q)={abs(math.degrees(side_q[2])):.1f}deg"
                )

        if self._max_elbow_height_m is not None:
            self._pin.forwardKinematics(self._model, self._data, q)
            self._pin.updateFramePlacements(self._model, self._data)
            for side in sorted(active_sides):
                height = float(
                    self._data.oMf[self._elbow_frames[side]].translation[2]
                )
                if height > self._max_elbow_height_m:
                    return (
                        f"{side} elbow exceeds height limit: "
                        f"z={height:.3f}m > {self._max_elbow_height_m:.3f}m"
                    )
        return None


def log_at_level(logger: Any, level: str, message: str) -> None:
    """Log on a severity-specific call site for ROS 2 Humble compatibility."""

    if level == "error":
        logger.error(message)
    elif level in {"warn", "warning"}:
        logger.warning(message)
    elif level == "info":
        logger.info(message)
    else:
        raise ValueError(f"unsupported log level: {level}")


@dataclass
class ArmFeedback:
    positions: np.ndarray
    received_s: float


@dataclass
class QuestHandStream:
    pose: np.ndarray | None = None
    pose_received_s: float = -math.inf
    source_timestamp_ns: int = 0
    grip: float = 0.0
    grip_received_s: float = -math.inf
    trigger: float = 0.0
    trigger_received_s: float = -math.inf
    rate: float = 0.1
    rate_received_s: float = -math.inf


@dataclass
class QuestPoseStream:
    pose: np.ndarray | None = None
    received_s: float = -math.inf
    source_timestamp_ns: int = 0


def freshest_trigger(
    streams: tuple[QuestHandStream, ...], now_s: float, timeout_s: float
) -> float | None:
    candidates = [
        stream
        for stream in streams
        if now_s - stream.trigger_received_s <= timeout_s
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda item: item.trigger_received_s).trigger


def build_button_motion_target(name: str, q_upper: Any) -> np.ndarray:
    """Build the NERO B/Y 14-joint model target."""

    target = np.zeros(14, dtype=np.float64)
    if name == "go_home":
        return target
    if name == "hands_up":
        upper = np.asarray(q_upper, dtype=np.float64)
        if upper.shape != (14,) or not np.all(np.isfinite(upper)):
            raise ValueError("OpenArmX upper limits must contain 14 finite values")
        target[3] = min(HANDS_UP_J4_RAD, float(upper[3]))
        target[10] = min(HANDS_UP_J4_RAD, float(upper[10]))
        return target
    raise ValueError(f"unsupported button motion: {name}")


def limit_button_motion_step(
    target: Any, current_command: Any, step_rad: float
) -> np.ndarray:
    """Apply the official fixed per-cycle step used by B/Y motions."""

    desired = np.asarray(target, dtype=np.float64)
    current = np.asarray(current_command, dtype=np.float64)
    step = float(step_rad)
    if desired.shape != (14,) or current.shape != (14,):
        raise ValueError("button motion target and command must contain 14 joints")
    if not np.all(np.isfinite(desired)) or not np.all(np.isfinite(current)):
        raise ValueError("button motion joints must be finite")
    if not math.isfinite(step) or step <= 0.0:
        raise ValueError("button motion step must be positive and finite")
    return current + np.clip(desired - current, -step, step)


def neutral_relative_pose(pose_type: Any, timestamp_s: float) -> Any:
    """Build a fresh zero-delta pose so the OpenArmX core observes release."""

    timestamp = float(timestamp_s)
    if not math.isfinite(timestamp):
        raise ValueError("relative release timestamp must be finite")
    return pose_type(
        position=np.zeros(3, dtype=np.float64),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        timestamp=timestamp,
    )


def reset_relative_handoff(
    core: Any,
    pose_type: Any,
    relative_frame_type: Any,
    absolute_frame_type: Any,
    model_feedback: Any,
    timestamp_s: float,
) -> None:
    """Release both relative arms and seed IK from measured joint feedback."""

    current = np.asarray(model_feedback, dtype=np.float64)
    if current.shape != (14,) or not np.all(np.isfinite(current)):
        raise ValueError("relative handoff feedback must contain 14 finite joints")
    release_pose = neutral_relative_pose(pose_type, timestamp_s)
    core.update_joint_states(OPENARMX_JOINT_NAMES, current)
    core.step(
        relative_frame_type(
            left_pose=release_pose,
            left_grip=0.0,
            right_pose=release_pose,
            right_grip=0.0,
        ),
        absolute_frame_type(),
    )
    core.ik_solver.reset(current)


def _make_core(
    model_path: Path,
    input_timeout_s: float,
    grip_threshold: float = 0.5,
    position_scale: float = 0.5,
    input_yaw_deg: float = 0.0,
    joint_regularization_weight: float = 0.2,
    joint3_regularization_weight: float = 0.05,
    joint4_regularization_weight: float = 0.12,
) -> tuple[Any, Any, Any, Any]:
    from openarmx_arm_driver import (
        AbsoluteTeleopInputFrame,
        PinocchioTeleopCore,
        PoseInput,
        RelativeTeleopInputFrame,
        TeleopConfig,
        WeightedMovingFilter,
    )

    # Match the official OpenArmX absolute-mode defaults. Relative Quest input
    # is clutch-anchored and mapped separately with the official mode matrix.
    absolute_position_axis = yaw_adjusted_axis(
        NERO_TCP_TRANSLATION_AXIS,
        input_yaw_deg,
    )
    absolute_orientation_axis = yaw_adjusted_axis(
        NERO_TCP_ORIENTATION_AXIS,
        input_yaw_deg,
    )
    config = TeleopConfig(
        urdf_path=str(model_path),
        constraint_mode="joint",
        use_link4_ext=False,
        grip_threshold=float(grip_threshold),
        print_performance=False,
        enable_placo_viewer=False,
        body_anchor_offset=np.asarray([0.0, -0.18, 0.08], dtype=np.float64),
        position_scale_xyz=np.asarray(
            [position_scale, position_scale, 0.9 * position_scale],
            dtype=np.float64,
        ),
        left_axis_matrix=absolute_position_axis,
        right_axis_matrix=absolute_position_axis,
        left_orientation_matrix=absolute_orientation_axis,
        right_orientation_matrix=absolute_orientation_axis,
        stream_timeout_sec=float(input_timeout_s),
        joint_regularization_weight=float(joint_regularization_weight),
        joint3_regularization_weight=float(joint3_regularization_weight),
        joint4_regularization_weight=float(joint4_regularization_weight),
    )
    core = PinocchioTeleopCore(config)
    core.ik_solver.q_filter = WeightedMovingFilter(
        OPENARMX_IK_FILTER_WEIGHTS.copy(), 14
    )
    return (
        core,
        PoseInput,
        RelativeTeleopInputFrame,
        AbsoluteTeleopInputFrame,
    )


def check_model(model_path: Path, input_timeout_s: float) -> None:
    core, _pose_type, _relative_type, _absolute_type = _make_core(
        model_path, input_timeout_s
    )
    zero_hardware = {
        "arm_a": np.zeros(7, dtype=np.float64),
        "arm_b": np.zeros(7, dtype=np.float64),
    }
    model_q = hardware_to_model(zero_hardware)
    core.update_joint_states(OPENARMX_JOINT_NAMES, model_q)
    current = np.asarray(core.get_current_joint_positions(), dtype=np.float64)
    if current.shape != (14,) or not np.allclose(current, model_q):
        raise RuntimeError("OpenArmX core did not accept the NERO joint model")
    left_tcp, right_tcp = core.ik_solver.get_end_effector_transforms(current)
    if np.asarray(left_tcp).shape != (4, 4) or np.asarray(right_tcp).shape != (4, 4):
        raise RuntimeError("OpenArmX core did not resolve both NERO TCP frames")
    print(
        "[check] OpenArmX Pinocchio core loaded dual NERO model: "
        f"left(arm_b)={np.asarray(left_tcp)[:3, 3].round(4).tolist()} "
        f"right(arm_a)={np.asarray(right_tcp)[:3, 3].round(4).tolist()}"
    )


def _node_class() -> Any:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Float32, String

    class NeroXOpenArmXTeleopNode(Node):
        def __init__(self, args: argparse.Namespace) -> None:
            super().__init__("nerox_openarmx_quest_teleop")
            self.args = args
            self.execute = bool(args.execute)
            (
                self.core,
                self.pose_type,
                self.relative_frame_type,
                self.absolute_frame_type,
            ) = _make_core(
                args.model,
                args.input_timeout_s,
                args.grip_threshold,
                args.position_scale,
                args.input_yaw_deg,
                args.ik_joint_regularization_weight,
                args.ik_joint3_regularization_weight,
                args.ik_joint4_regularization_weight,
            )
            self.posture_guard = LowElbowPostureGuard(
                args.model,
                max_elbow_height_m=args.posture_max_elbow_height_m,
                max_abs_j2_deg=args.posture_max_abs_j2_deg,
                max_abs_j3_deg=args.posture_max_abs_j3_deg,
                joint_limit_margin_deg=args.posture_joint_limit_margin_deg,
            )

            position_axis = (
                NERO_TCP_TRANSLATION_AXIS
                if args.controller_pose_mode == "gripper"
                else QUEST_HAND_AXIS
            )
            position_axis = yaw_adjusted_axis(
                position_axis,
                args.input_yaw_deg,
            )
            orientation_axis = yaw_adjusted_axis(
                NERO_TCP_ORIENTATION_AXIS,
                args.input_yaw_deg,
            )
            self.trackers = {
                side: RelativePoseTracker(
                    self.pose_type,
                    position_axis,
                    orientation_axis,
                )
                for side in ("left", "right")
            }
            self.limiter = OpenArmXJointStepLimiter()
            self.feedback: dict[str, ArmFeedback] = {}
            self.quest_relative = {
                side: QuestHandStream() for side in ("left", "right")
            }
            self.quest_absolute = {
                side: QuestHandStream() for side in ("left", "right")
            }
            self.quest_head = QuestPoseStream()
            self._pose_mode_ok = True
            self._active_sides: set[str] = set()
            self._active_mode: str | None = None
            self._last_status_s = time.monotonic()
            self._last_log: dict[str, float] = {}
            self._solve_count = 0
            self._publish_count = 0
            self._gripper_command_count = 0
            self._button_pressed = {
                "relative_b": False,
                "relative_y": False,
                "absolute_b": False,
                "absolute_y": False,
            }
            self._pending_go_home = False
            self._pending_hands_up = False
            self._button_motion_name: str | None = None
            self._button_motion_target: np.ndarray | None = None
            self._button_motion_command: np.ndarray | None = None
            self._button_motion_reached_cycles = 0
            self.tacclaw = None
            self.tacclaw_state_publisher = None
            self.tacclaw_command_publisher = None
            self.tacclaw_applied_publishers = {}

            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.feedback_subscriptions = {
                arm: self.create_subscription(
                    JointState,
                    f"/{arm}/feedback/joint_states",
                    lambda message, name=arm: self._feedback_callback(name, message),
                    qos,
                )
                for arm in ("arm_a", "arm_b")
            }
            self.command_publishers = {
                arm: self.create_publisher(
                    JointState,
                    f"/{arm}/control/joint_states",
                    qos,
                )
                for arm in ("arm_a", "arm_b")
            }
            self.position_command_publishers = {
                arm: self.create_publisher(JointState, f"/{arm}/control/move_j", qos)
                for arm in ("arm_a", "arm_b")
            }
            self.command_recording_publishers = {
                arm: self.create_publisher(JointState, f"/nerox/{arm}/command", qos)
                for arm in ("arm_a", "arm_b")
            }
            self.quest_subscriptions = []
            self._setup_quest_subscriptions(PoseStamped, Float32, Bool, String, qos)

            if args.with_tacclaw:
                from nerox_openarmx_teleop.tacclaw import TacClawPair

                self.tacclaw_state_publisher = self.create_publisher(
                    JointState, TACCLAW_STATE_TOPIC, qos
                )
                self.tacclaw_command_publisher = self.create_publisher(
                    JointState, TACCLAW_COMMAND_TOPIC, qos
                )
                self.tacclaw_applied_publishers = {
                    side: self.create_publisher(JointState, topic, qos)
                    for side, topic in TACCLAW_APPLIED_TOPICS.items()
                }
                try:
                    self.tacclaw = TacClawPair(args)
                except Exception as exc:
                    self._log_throttled(
                        "tacclaw_startup",
                        "error",
                        "TacClaw disabled; NERO arms remain available: "
                        f"{type(exc).__name__}: {exc}",
                    )

            self.timer = self.create_timer(1.0 / args.control_rate_hz, self._tick)
            mode = "EXECUTE" if self.execute else "SHADOW"
            gripper = "dual TacClaw" if self.tacclaw is not None else "arm-only"
            input_label = (
                "official OpenArmX Quest topics"
                if args.input_backend == "openarmx_udp"
                else "XRobotToolkit OpenArmX compatibility topics"
            )
            self.get_logger().info(
                f"{mode}: {input_label} + OpenArmX IK at "
                f"{args.control_rate_hz:.0f}Hz; {gripper}; left->arm_b right->arm_a; "
                "ik_backend=pinocchio; "
                f"position_scale={args.position_scale:.2f}; "
                f"input_yaw={args.input_yaw_deg:g}deg; "
                "tcp_translation=nero; tcp_rotation=nero; "
                "ik_joint_filter=official-wma-8frame; "
                "tacclaw_trigger=independent-of-grip; "
                "joint_step=official(slow=4deg;fast=20/16/12/12/14/8/12deg)"
            )
            if self.execute:
                self.get_logger().warn(
                    "Live output enabled; Grip above the OpenArmX threshold takes over immediately"
                )

        def _setup_quest_subscriptions(
            self,
            pose_message: Any,
            float_message: Any,
            bool_message: Any,
            string_message: Any,
            qos: Any,
        ) -> None:
            for side in ("left", "right"):
                relative = self.quest_relative[side]
                absolute = self.quest_absolute[side]
                self.quest_subscriptions.extend(
                    [
                        self.create_subscription(
                            pose_message,
                            f"/pico_{side}_controller/pose",
                            lambda msg, stream=relative, label=f"relative {side}": self._pose_callback(
                                stream, msg, label
                            ),
                            qos,
                        ),
                        self.create_subscription(
                            float_message,
                            f"/pico_{side}_controller/grip",
                            lambda msg, stream=relative, label=f"relative {side} grip": self._scalar_callback(
                                stream, "grip", msg.data, label
                            ),
                            qos,
                        ),
                        self.create_subscription(
                            float_message,
                            f"/pico_{side}_controller/trigger",
                            lambda msg, stream=relative, label=f"relative {side} trigger": self._scalar_callback(
                                stream, "trigger", msg.data, label
                            ),
                            qos,
                        ),
                        self.create_subscription(
                            float_message,
                            f"/pico_{side}_controller/rate",
                            lambda msg, stream=relative, label=f"relative {side} rate": self._scalar_callback(
                                stream, "rate", msg.data, label
                            ),
                            qos,
                        ),
                        self.create_subscription(
                            pose_message,
                            f"/vr/{side}/pose_absolute",
                            lambda msg, stream=absolute, label=f"absolute {side}": self._pose_callback(
                                stream, msg, label
                            ),
                            qos,
                        ),
                        self.create_subscription(
                            float_message,
                            f"/vr/{side}/grip",
                            lambda msg, stream=absolute, label=f"absolute {side} grip": self._scalar_callback(
                                stream, "grip", msg.data, label
                            ),
                            qos,
                        ),
                        self.create_subscription(
                            float_message,
                            f"/vr/{side}/trigger",
                            lambda msg, stream=absolute, label=f"absolute {side} trigger": self._scalar_callback(
                                stream, "trigger", msg.data, label
                            ),
                            qos,
                        ),
                    ]
                )
            self.quest_subscriptions.extend(
                [
                    self.create_subscription(
                        pose_message,
                        "/vr/head/pose",
                        self._head_pose_callback,
                        qos,
                    ),
                    self.create_subscription(
                        float_message,
                        "/vr/rate",
                        self._absolute_rate_callback,
                        qos,
                    ),
                    self.create_subscription(
                        string_message,
                        "/vr/control_mode",
                        self._absolute_mode_callback,
                        qos,
                    ),
                    self.create_subscription(
                        bool_message,
                        "/vr/calibrate_done",
                        self._absolute_calibrate_callback,
                        qos,
                    ),
                    self.create_subscription(
                        string_message,
                        "/vr/controller_pose_mode",
                        self._pose_source_mode_callback,
                        qos,
                    ),
                    self.create_subscription(
                        bool_message,
                        "/pico_right_controller/button_b",
                        lambda msg: self._button_callback(
                            "relative_b", "go_home", msg
                        ),
                        qos,
                    ),
                    self.create_subscription(
                        bool_message,
                        "/pico_left_controller/button_y",
                        lambda msg: self._button_callback(
                            "relative_y", "hands_up", msg
                        ),
                        qos,
                    ),
                    self.create_subscription(
                        bool_message,
                        "/vr/right/button_b",
                        lambda msg: self._button_callback(
                            "absolute_b", "go_home", msg
                        ),
                        qos,
                    ),
                    self.create_subscription(
                        bool_message,
                        "/vr/left/button_y",
                        lambda msg: self._button_callback(
                            "absolute_y", "hands_up", msg
                        ),
                        qos,
                    ),
                ]
            )

        @staticmethod
        def _message_timestamp_ns(message: Any) -> int:
            stamp = message.header.stamp
            return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

        def _pose_callback(
            self, stream: QuestHandStream, message: Any, label: str
        ) -> None:
            try:
                pose = validated_pose(
                    [
                        message.pose.position.x,
                        message.pose.position.y,
                        message.pose.position.z,
                        message.pose.orientation.x,
                        message.pose.orientation.y,
                        message.pose.orientation.z,
                        message.pose.orientation.w,
                    ],
                    label,
                )
            except ValueError as exc:
                self._log_throttled(f"invalid_{label}", "error", str(exc))
                return
            stream.pose = pose
            stream.pose_received_s = time.monotonic()
            stream.source_timestamp_ns = self._message_timestamp_ns(message)

        def _head_pose_callback(self, message: Any) -> None:
            try:
                pose = validated_pose(
                    [
                        message.pose.position.x,
                        message.pose.position.y,
                        message.pose.position.z,
                        message.pose.orientation.x,
                        message.pose.orientation.y,
                        message.pose.orientation.z,
                        message.pose.orientation.w,
                    ],
                    "absolute head",
                )
            except ValueError as exc:
                self._log_throttled("invalid_absolute_head", "error", str(exc))
                return
            self.quest_head.pose = pose
            self.quest_head.received_s = time.monotonic()
            self.quest_head.source_timestamp_ns = self._message_timestamp_ns(message)

        def _scalar_callback(
            self, stream: QuestHandStream, field: str, value: float, label: str
        ) -> None:
            try:
                resolved = bounded_unit(value, label)
            except ValueError as exc:
                self._log_throttled(f"invalid_{label}", "error", str(exc))
                return
            setattr(stream, field, resolved)
            setattr(stream, f"{field}_received_s", time.monotonic())

        def _absolute_rate_callback(self, message: Any) -> None:
            now_s = time.monotonic()
            try:
                rate = bounded_unit(message.data, "absolute rate")
            except ValueError as exc:
                self._log_throttled("invalid_absolute_rate", "error", str(exc))
                return
            for stream in self.quest_absolute.values():
                stream.rate = rate
                stream.rate_received_s = now_s

        def _absolute_mode_callback(self, message: Any) -> None:
            try:
                self.core.update_absolute_mode(str(message.data))
            except Exception as exc:
                self._log_throttled(
                    "absolute_mode", "error", f"invalid Quest control mode: {exc}"
                )

        def _absolute_calibrate_callback(self, message: Any) -> None:
            if not bool(message.data):
                return
            frame = self._absolute_frame(time.monotonic(), calibration=True)
            if self.core.calibrate_absolute(frame):
                self.get_logger().info("OpenArmX absolute calibration completed")
            else:
                self.get_logger().warn(
                    "OpenArmX absolute calibration skipped: poses/mode incomplete"
                )

        def _pose_source_mode_callback(self, message: Any) -> None:
            received = str(message.data).strip().lower()
            self._pose_mode_ok = received == self.args.controller_pose_mode
            if not self._pose_mode_ok:
                self._fault_reset()
                self._log_throttled(
                    "pose_mode_mismatch",
                    "error",
                    f"Quest pose mode is {received!r}, node expects "
                    f"{self.args.controller_pose_mode!r}",
                )

        def _button_callback(
            self, state_name: str, motion_name: str, message: Any
        ) -> None:
            pressed = bool(message.data)
            previous = self._button_pressed[state_name]
            self._button_pressed[state_name] = pressed
            if pressed and not previous:
                if motion_name == "go_home":
                    self._pending_go_home = True
                else:
                    self._pending_hands_up = True

        def _feedback_callback(self, arm: str, message: Any) -> None:
            positions = {name: value for name, value in zip(message.name, message.position)}
            try:
                vector = np.asarray(
                    [positions[name] for name in HARDWARE_JOINT_NAMES],
                    dtype=np.float64,
                )
            except KeyError as exc:
                self._log_throttled(
                    f"feedback_names_{arm}",
                    "error",
                    f"{arm} feedback is missing {exc.args[0]}",
                )
                return
            if vector.shape != (7,) or not np.all(np.isfinite(vector)):
                self._log_throttled(
                    f"feedback_values_{arm}",
                    "error",
                    f"{arm} feedback contains invalid positions",
                )
                return
            self.feedback[arm] = ArmFeedback(vector, time.monotonic())

        def _fresh_hardware(self, now_s: float) -> dict[str, np.ndarray] | None:
            missing = [arm for arm in ("arm_a", "arm_b") if arm not in self.feedback]
            if missing:
                self._log_throttled(
                    "feedback_missing",
                    "warn",
                    "waiting for NERO feedback: " + ", ".join(missing),
                )
                return None
            stale = [
                arm
                for arm, feedback in self.feedback.items()
                if now_s - feedback.received_s > self.args.robot_timeout_s
            ]
            if stale:
                self._fault_reset()
                self._log_throttled(
                    "feedback_stale",
                    "error",
                    "stale NERO feedback: " + ", ".join(stale),
                )
                return None
            return {
                arm: self.feedback[arm].positions.copy() for arm in ("arm_a", "arm_b")
            }

        def _fresh(self, received_s: float, now_s: float) -> bool:
            return now_s - received_s <= self.args.input_timeout_s

        def _pose_input(self, pose: np.ndarray, timestamp_s: float) -> Any:
            return self.pose_type(
                position=np.asarray(pose[:3], dtype=np.float64),
                orientation_xyzw=tuple(float(value) for value in pose[3:]),
                timestamp=float(timestamp_s),
            )

        def _relative_frame(
            self,
            now_s: float,
            robot_tcp_rotations: dict[str, np.ndarray],
        ) -> Any:
            values: dict[str, Any] = {}
            for side, stream in self.quest_relative.items():
                input_fresh = (
                    stream.pose is not None
                    and self._fresh(stream.pose_received_s, now_s)
                    and self._fresh(stream.grip_received_s, now_s)
                )
                if input_fresh:
                    authorized = stream.grip > self.args.grip_threshold
                    if authorized:
                        pose = self.trackers[side].update(
                            stream.pose,
                            True,
                            stream.pose_received_s,
                            robot_tcp_rotations[side],
                        )
                    else:
                        self.trackers[side].reset()
                        pose = neutral_relative_pose(self.pose_type, now_s)
                else:
                    self.trackers[side].reset()
                    authorized = False
                    pose = neutral_relative_pose(self.pose_type, now_s)
                values[f"{side}_pose"] = pose
                values[f"{side}_grip"] = 1.0 if authorized else 0.0
                values[f"{side}_trigger"] = (
                    stream.trigger
                    if self._fresh(stream.trigger_received_s, now_s)
                    else 0.0
                )
            return self.relative_frame_type(**values)

        def _absolute_frame(self, now_s: float, calibration: bool = False) -> Any:
            head = None
            if self.quest_head.pose is not None and self._fresh(
                self.quest_head.received_s, now_s
            ):
                head = self._pose_input(
                    self.quest_head.pose, self.quest_head.received_s
                )
            values: dict[str, Any] = {"head_pose": head}
            for side, stream in self.quest_absolute.items():
                pose_fresh = stream.pose is not None and self._fresh(
                    stream.pose_received_s, now_s
                )
                grip_fresh = self._fresh(stream.grip_received_s, now_s)
                pose = (
                    self._pose_input(stream.pose, stream.pose_received_s)
                    if pose_fresh
                    else None
                )
                if calibration:
                    grip = stream.grip if grip_fresh else 0.0
                elif pose_fresh and grip_fresh:
                    grip = stream.grip
                else:
                    grip = 0.0
                values[f"{side}_pose"] = pose
                values[f"{side}_grip"] = grip
                values[f"{side}_trigger"] = (
                    stream.trigger
                    if self._fresh(stream.trigger_received_s, now_s)
                    else 0.0
                )
            values["mode_selected"] = bool(self.core.absolute_mode_selected)
            return self.absolute_frame_type(**values)

        def _stream(self, mode: str, side: str) -> QuestHandStream:
            return (
                self.quest_absolute[side]
                if mode == "absolute"
                else self.quest_relative[side]
            )

        def _command_fresh_tacclaw_triggers(self, now_s: float) -> None:
            if self.tacclaw is None:
                return
            for side in ("left", "right"):
                trigger = freshest_trigger(
                    (
                        self.quest_relative[side],
                        self.quest_absolute[side],
                    ),
                    now_s,
                    self.args.input_timeout_s,
                )
                if trigger is None:
                    continue
                self.tacclaw.command(side, trigger)
                self._gripper_command_count += 1

        def _manual_override_requested(self, now_s: float) -> bool:
            for streams in (self.quest_relative, self.quest_absolute):
                for stream in streams.values():
                    if (
                        self._fresh(stream.grip_received_s, now_s)
                        and stream.grip > self.args.grip_threshold
                    ):
                        return True
            return False

        def _reset_relative_handoff(
            self,
            model_feedback: np.ndarray,
            now_s: float,
        ) -> None:
            reset_relative_handoff(
                self.core,
                self.pose_type,
                self.relative_frame_type,
                self.absolute_frame_type,
                model_feedback,
                now_s,
            )
            for tracker in self.trackers.values():
                tracker.reset()

        def _start_button_motion(
            self,
            name: str,
            model_feedback: np.ndarray,
            now_s: float,
        ) -> None:
            self._reset_relative_handoff(model_feedback, now_s)
            self._button_motion_name = name
            self._button_motion_target = build_button_motion_target(
                name, self.core.ik_solver.q_upper
            )
            self._button_motion_command = None
            self._button_motion_reached_cycles = 0
            for tracker in self.trackers.values():
                tracker.reset()
            self._active_mode = f"button:{name}"
            self._active_sides = {"left", "right"}
            if self.tacclaw is not None:
                self.tacclaw.cancel_pending()
            self.get_logger().info(
                f"Starting official OpenArmX button motion '{name}' at "
                f"{self.args.button_motion_step_deg:.2f}deg/cycle; "
                "transport=move_j position control"
            )

        def _cancel_button_motion(self, reason: str) -> None:
            if self._button_motion_name is not None:
                self.get_logger().info(
                    f"Cancelling button motion '{self._button_motion_name}': {reason}"
                )
            self._button_motion_name = None
            self._button_motion_target = None
            self._button_motion_command = None
            self._button_motion_reached_cycles = 0

        def _process_button_events(
            self,
            model_feedback: np.ndarray,
            now_s: float,
        ) -> None:
            pending_go_home = self._pending_go_home
            pending_hands_up = self._pending_hands_up
            self._pending_go_home = False
            self._pending_hands_up = False
            if pending_go_home:
                self._start_button_motion("go_home", model_feedback, now_s)
            if pending_hands_up:
                self._start_button_motion("hands_up", model_feedback, now_s)

        def _run_button_motion(
            self,
            model_feedback: np.ndarray,
            now_s: float,
        ) -> None:
            if self._button_motion_name is None or self._button_motion_target is None:
                return
            if self._manual_override_requested(now_s):
                self._reset_relative_handoff(model_feedback, now_s)
                self._cancel_button_motion("Grip manual override")
                self._active_mode = None
                self._active_sides.clear()
                return
            if self._button_motion_command is None:
                self._button_motion_command = model_feedback.copy()
            command_model = limit_button_motion_step(
                self._button_motion_target,
                self._button_motion_command,
                math.radians(self.args.button_motion_step_deg),
            )
            command_hardware = model_to_hardware(command_model)
            if self.execute:
                for arm in ("arm_a", "arm_b"):
                    self._publish_command(
                        arm,
                        command_hardware[arm],
                        0,
                        f"button_{self._button_motion_name}",
                        position_mode=True,
                    )
            self._button_motion_command = command_model
            max_error = float(
                np.max(np.abs(self._button_motion_target - model_feedback))
            )
            if max_error <= math.radians(
                self.args.button_motion_done_tolerance_deg
            ):
                self._button_motion_reached_cycles += 1
            else:
                self._button_motion_reached_cycles = 0
            if self._button_motion_reached_cycles >= 5:
                name = self._button_motion_name
                self._reset_relative_handoff(model_feedback, now_s)
                self._cancel_button_motion("target reached")
                self._active_mode = None
                self._active_sides.clear()
                self.get_logger().info(f"Button motion '{name}' completed")
            self._report_status(now_s)

        def _tick(self) -> None:
            now_s = time.monotonic()
            if not self._pose_mode_ok:
                return
            if self.tacclaw is not None:
                try:
                    self._publish_tacclaw_snapshot()
                    fault = self.tacclaw.fault_message()
                    if fault is not None:
                        self._log_throttled(
                            "tacclaw_fault",
                            "error",
                            "TacClaw output held; NERO arms remain available: "
                            f"{fault}",
                        )
                    elif not self.tacclaw.ready:
                        self._log_throttled(
                            "tacclaw_connecting",
                            "warn",
                            "waiting for both TacClaw workers; "
                            "NERO arms remain available",
                        )
                    else:
                        self._command_fresh_tacclaw_triggers(now_s)
                except Exception as exc:
                    self._log_throttled(
                        "tacclaw_command",
                        "error",
                        "TacClaw command held; NERO arms remain available: "
                        f"{type(exc).__name__}: {exc}",
                    )
            hardware = self._fresh_hardware(now_s)
            if hardware is None:
                return
            try:
                model_feedback = hardware_to_model(hardware)
                self.core.update_joint_states(OPENARMX_JOINT_NAMES, model_feedback)
                left_tcp, right_tcp = self.core.ik_solver.get_end_effector_transforms(
                    model_feedback
                )
                robot_tcp_rotations = {
                    "left": np.asarray(left_tcp, dtype=np.float64)[:3, :3],
                    "right": np.asarray(right_tcp, dtype=np.float64)[:3, :3],
                }
                self._process_button_events(model_feedback, now_s)
                if self._button_motion_name is not None:
                    self._run_button_motion(model_feedback, now_s)
                    return
                result = self.core.step(
                    self._relative_frame(now_s, robot_tcp_rotations),
                    self._absolute_frame(now_s),
                )
                if result.error:
                    raise RuntimeError(f"OpenArmX teleop core: {result.error}")
                active_mode = result.active_mode
                active_sides = {
                    side
                    for side, active in (
                        ("left", result.left_active),
                        ("right", result.right_active),
                    )
                    if active
                }
                self._active_sides = active_sides
                self._active_mode = active_mode
                if result.target_q is None or not active_sides or active_mode is None:
                    self._report_status(now_s)
                    return

                posture_rejection = self.posture_guard.rejection(
                    result.target_q, active_sides
                )
                if posture_rejection is not None:
                    self._log_throttled(
                        "posture_guard",
                        "warn",
                        f"posture guard held previous safe command: {posture_rejection}",
                    )
                    self._report_status(now_s)
                    return

                target_by_arm = model_to_hardware(result.target_q)
                for side in active_sides:
                    arm = OPENARMX_SIDE_TO_HARDWARE_ARM[side]
                    stream = self._stream(active_mode, side)
                    fast_mode = (
                        self._fresh(stream.rate_received_s, now_s)
                        and stream.rate >= 0.999
                    )
                    command = self.limiter.limit(
                        target_by_arm[arm],
                        hardware[arm],
                        fast_mode=fast_mode,
                    )
                    if self.execute:
                        self._publish_command(
                            arm, command, stream.source_timestamp_ns, active_mode
                        )
                self._solve_count += 1
                self._report_status(now_s)
            except Exception as exc:
                self._fault_reset()
                self._log_throttled(
                    "control_fault",
                    "error",
                    f"control held: {type(exc).__name__}: {exc}",
                )

        def _publish_command(
            self,
            arm: str,
            positions: np.ndarray,
            source_timestamp_ns: int,
            active_mode: str,
            *,
            position_mode: bool = False,
        ) -> None:
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = (
                f"openarmx_quest_{active_mode};source={source_timestamp_ns}"
            )
            message.name = list(HARDWARE_JOINT_NAMES)
            message.position = [float(value) for value in positions]
            # Preset poses need the firmware position controller. Fast JS/MIT
            # tracking left a measured 2.1-degree J4 error under the wrist load.
            publishers = (
                self.position_command_publishers if position_mode
                else self.command_publishers
            )
            publishers[arm].publish(message)
            self.command_recording_publishers[arm].publish(message)
            self._publish_count += 1

        def _publish_tacclaw_snapshot(self) -> None:
            if self.tacclaw is None:
                return
            snapshot = self.tacclaw.snapshot()
            self._publish_tacclaw_vector(
                self.tacclaw_state_publisher,
                snapshot,
                value_key="measured_position",
                frame_id="nerox_tacclaw_measured",
            )
            self._publish_tacclaw_vector(
                self.tacclaw_command_publisher,
                snapshot,
                value_key="commanded_position",
                frame_id="nerox_tacclaw_commanded",
            )
            self._publish_tacclaw_applied(snapshot)

        def _publish_tacclaw_applied(
            self,
            snapshot: dict[str, dict[str, Any]],
        ) -> None:
            for side, publisher in self.tacclaw_applied_publishers.items():
                item = snapshot[side]
                required = (
                    item.get("commanded_position"),
                    item.get("command_sequence"),
                    item.get("command_send_wall_ns"),
                    item.get("command_send_monotonic_ns"),
                    item.get("command_ack_wall_ns"),
                    item.get("command_ack_monotonic_ns"),
                )
                if any(value is None for value in required):
                    continue
                position, sequence, send_wall_ns, send_mono_ns, ack_wall_ns, ack_mono_ns = (
                    int(value) for value in required
                )
                message = JointState()
                message.header.stamp.sec = send_wall_ns // 1_000_000_000
                message.header.stamp.nanosec = send_wall_ns % 1_000_000_000
                message.header.frame_id = (
                    "nerox_tacclaw_applied;"
                    f"side={side};sequence={sequence};"
                    f"send_monotonic_ns={send_mono_ns};"
                    f"ack_wall_ns={ack_wall_ns};"
                    f"ack_monotonic_ns={ack_mono_ns}"
                )
                message.name = [f"{side}_tacclaw_position"]
                message.position = [float(position)]
                message.effort = [1.0 if item["ready"] else 0.0]
                # Repeat the latest acknowledged event so a recorder starting
                # after teleop still proves readiness. The raw writer dedupes
                # by exact source stamp and frame id.
                publisher.publish(message)

        def _publish_tacclaw_vector(
            self,
            publisher: Any,
            snapshot: dict[str, dict[str, Any]],
            *,
            value_key: str,
            frame_id: str,
        ) -> None:
            if publisher is None:
                return
            values = [snapshot[side][value_key] for side in ("left", "right")]
            if any(value is None for value in values):
                return
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = frame_id
            message.name = [
                "left_tacclaw_position",
                "right_tacclaw_position",
            ]
            message.position = [float(value) for value in values]
            message.effort = [
                1.0 if snapshot[side]["ready"] else 0.0
                for side in ("left", "right")
            ]
            publisher.publish(message)

        def _fault_reset(self) -> None:
            self._pending_go_home = False
            self._pending_hands_up = False
            self._cancel_button_motion("control reset")
            for tracker in self.trackers.values():
                tracker.reset()
            self._active_sides.clear()
            self._active_mode = None
            if self.tacclaw is not None:
                try:
                    self.tacclaw.cancel_pending()
                except Exception as exc:
                    self._log_throttled(
                        "tacclaw_cancel",
                        "error",
                        "TacClaw cancel failed; NERO arms remain available: "
                        f"{type(exc).__name__}: {exc}",
                    )

        def _report_status(self, now_s: float) -> None:
            if now_s - self._last_status_s < 5.0:
                return
            self._last_status_s = now_s
            mode = "execute" if self.execute else "shadow"
            active = ",".join(sorted(self._active_sides)) or "none"
            input_mode = self._active_mode or "idle"
            self.get_logger().info(
                f"mode={mode} quest={input_mode} active={active} "
                f"solves={self._solve_count} publishes={self._publish_count} "
                f"tacclaw={self._gripper_command_count}"
            )

        def _log_throttled(
            self, key: str, level: str, message: str, period_s: float = 2.0
        ) -> None:
            now_s = time.monotonic()
            if now_s - self._last_log.get(key, -math.inf) < period_s:
                return
            self._last_log[key] = now_s
            log_at_level(self.get_logger(), level, message)

        def close(self) -> None:
            self._fault_reset()
            if self.tacclaw is not None:
                self.tacclaw.close()

    return NeroXOpenArmXTeleopNode


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    root = Path(__file__).resolve().parents[2]
    token_default = Path(
        os.environ.get(
            "NEROX_TACCLAW_TOKEN_PATH", root / "secrets/tacclaw.token"
        )
    )
    parser = argparse.ArgumentParser(
        description="Dual-NERO teleoperation using official OpenArmX Quest input and IK"
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="validate model/core only")
    mode.add_argument("--shadow", action="store_true", help="solve but do not publish")
    mode.add_argument("--execute", action="store_true", help="publish live NERO commands")
    parser.add_argument(
        "--model",
        type=Path,
        default=root / "models/nerox_openarmx_bimanual.urdf",
    )
    parser.add_argument(
        "--controller-pose-mode",
        choices=("gripper", "hand"),
        default="gripper",
        help="must match AIM/GRIP mode selected in the Quest application",
    )
    parser.add_argument(
        "--input-backend",
        choices=("openarmx_udp", "xrobotoolkit"),
        default="openarmx_udp",
        help="owner of the OpenArmX-compatible XR input topics",
    )
    parser.add_argument("--control-rate-hz", type=float, default=100.0)
    parser.add_argument("--input-timeout-s", type=float, default=0.3)
    parser.add_argument("--robot-timeout-s", type=float, default=0.25)
    parser.add_argument("--grip-threshold", type=float, default=0.5)
    parser.add_argument(
        "--position-scale",
        type=float,
        default=0.5,
        help="Cartesian position scale applied by OpenArmX (default: 0.5)",
    )
    parser.add_argument(
        "--input-yaw-deg",
        type=float,
        default=0.0,
        help="yaw offset for the XR input frame in robot world coordinates",
    )
    parser.add_argument("--ik-joint-regularization-weight", type=float, default=0.2)
    parser.add_argument("--ik-joint3-regularization-weight", type=float, default=0.05)
    parser.add_argument("--ik-joint4-regularization-weight", type=float, default=0.12)
    parser.add_argument("--posture-max-elbow-height-m", type=float, default=None)
    parser.add_argument("--posture-max-abs-j2-deg", type=float, default=None)
    parser.add_argument("--posture-max-abs-j3-deg", type=float, default=None)
    parser.add_argument("--posture-joint-limit-margin-deg", type=float, default=0.0)
    parser.add_argument("--button-motion-step-deg", type=float, default=0.5)
    parser.add_argument(
        "--button-motion-done-tolerance-deg", type=float, default=1.0
    )

    parser.add_argument(
        "--with-tacclaw",
        action="store_true",
        help="enable both physical TacClaws (execute mode only)",
    )
    parser.add_argument(
        "--tacclaw-clearance-confirmed",
        action="store_true",
        help="confirm both grippers may initialize and move to their open position",
    )
    parser.add_argument("--tacclaw-token-path", type=Path, default=token_default)
    parser.add_argument("--tacclaw-left-host", default="192.168.127.10")
    parser.add_argument("--tacclaw-right-host", default="192.168.127.11")
    parser.add_argument("--tacclaw-port", type=int, default=55552)
    parser.add_argument("--tacclaw-left-closed-raw", type=int, default=269448)
    parser.add_argument("--tacclaw-left-open-raw", type=int, default=179448)
    parser.add_argument("--tacclaw-right-closed-raw", type=int, default=-406491)
    parser.add_argument("--tacclaw-right-open-raw", type=int, default=-496491)
    parser.add_argument("--tacclaw-speed", type=int, default=10)
    parser.add_argument("--tacclaw-torque-limit", type=int, default=20)
    parser.add_argument("--tacclaw-watchdog-s", type=float, default=0.35)
    parser.add_argument("--tacclaw-command-rate-hz", type=float, default=20.0)
    parser.add_argument("--tacclaw-position-deadband", type=int, default=3)
    parser.add_argument("--tacclaw-initial-position", type=int, default=1000)
    parser.add_argument("--tacclaw-connect-timeout-s", type=float, default=1.0)
    parser.add_argument("--tacclaw-request-timeout-s", type=float, default=0.75)
    parser.add_argument("--tacclaw-tracking-timeout-s", type=float, default=5.0)
    parser.add_argument("--tacclaw-tracking-tolerance", type=int, default=20)

    args = parser.parse_args(argv)
    args.model = args.model.expanduser().resolve()
    args.tacclaw_token_path = args.tacclaw_token_path.expanduser().resolve()
    if not args.model.is_file():
        parser.error(f"model does not exist: {args.model}")
    for name in (
        "control_rate_hz",
        "input_timeout_s",
        "robot_timeout_s",
        "position_scale",
        "button_motion_step_deg",
        "button_motion_done_tolerance_deg",
        "tacclaw_command_rate_hz",
        "tacclaw_connect_timeout_s",
        "tacclaw_request_timeout_s",
        "tacclaw_tracking_timeout_s",
    ):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive and finite")
    if not 0.0 < args.grip_threshold < 1.0:
        parser.error("--grip-threshold must be in (0, 1)")
    if args.position_scale > 1.0:
        parser.error("--position-scale must not exceed 1.0")
    if (
        not math.isfinite(args.input_yaw_deg)
        or not -180.0 <= args.input_yaw_deg <= 180.0
    ):
        parser.error("--input-yaw-deg must be finite and in [-180, 180]")
    for name in (
        "ik_joint_regularization_weight",
        "ik_joint3_regularization_weight",
        "ik_joint4_regularization_weight",
        "posture_joint_limit_margin_deg",
    ):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) < 0.0:
            parser.error(f"--{name.replace('_', '-')} must be finite and non-negative")
    if args.posture_joint_limit_margin_deg >= 45.0:
        parser.error("--posture-joint-limit-margin-deg must be less than 45")
    if args.posture_max_elbow_height_m is not None and not math.isfinite(
        args.posture_max_elbow_height_m
    ):
        parser.error("--posture-max-elbow-height-m must be finite")
    for name in ("posture_max_abs_j2_deg", "posture_max_abs_j3_deg"):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or not 0.0 < value <= 180.0):
            parser.error(f"--{name.replace('_', '-')} must be in (0, 180]")
    if not 1 <= args.tacclaw_port <= 65535:
        parser.error("--tacclaw-port must be in 1..65535")
    if not 10 <= args.tacclaw_speed <= 50:
        parser.error("--tacclaw-speed must be in 10..50")
    if not 10 <= args.tacclaw_torque_limit <= 20:
        parser.error("--tacclaw-torque-limit must be in 10..20")
    if not 0.2 <= args.tacclaw_watchdog_s <= 1.0:
        parser.error("--tacclaw-watchdog-s must be in 0.2..1.0")
    if not 0 <= args.tacclaw_position_deadband <= 1000:
        parser.error("--tacclaw-position-deadband must be in 0..1000")
    if not 0 <= args.tacclaw_initial_position <= 1000:
        parser.error("--tacclaw-initial-position must be in 0..1000")
    if not 1 <= args.tacclaw_tracking_tolerance <= 100:
        parser.error("--tacclaw-tracking-tolerance must be in 1..100")
    if args.with_tacclaw and not args.execute:
        parser.error("--with-tacclaw is only allowed together with --execute")
    if args.with_tacclaw and not args.tacclaw_clearance_confirmed:
        parser.error("--with-tacclaw requires --tacclaw-clearance-confirmed")
    if args.with_tacclaw and not args.tacclaw_token_path.is_file():
        parser.error(
            f"TacClaw token does not exist: {args.tacclaw_token_path}"
        )
    if not args.check and not args.execute:
        args.shadow = True
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.check:
        check_model(args.model, args.input_timeout_s)
        return 0

    import rclpy

    rclpy.init(args=None)
    node = _node_class()(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # SIGTERM can invalidate the ROS context while Humble is constructing
        # a wait set, producing either ExternalShutdownException or RCLError.
        if rclpy.ok():
            raise
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
