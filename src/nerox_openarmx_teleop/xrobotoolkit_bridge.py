"""XRoboToolkit PC Service -> OpenArmX-compatible ROS 2 topics.

Reads controller and headset poses from the native XRoboToolkit SDK and
publishes the `/pico_*` and `/vr/*` topics consumed by
`nerox_openarmx_teleop.node --input-backend xrobotoolkit`.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import time
from collections.abc import Sequence
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np

DEFAULT_POSES_TOPIC = "/nerox/xrobotoolkit/poses"
DEFAULT_TRIGGERS_TOPIC = "/nerox/xrobotoolkit/triggers"
DEFAULT_LEFT_CLUTCH_TOPIC = "/nerox/xrobotoolkit/left/clutch"
DEFAULT_RIGHT_CLUTCH_TOPIC = "/nerox/xrobotoolkit/right/clutch"
DEFAULT_SDK_LIBRARY_PATH = "/opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so"
DEFAULT_OPENARMX_TOPICS = {
    "left_pose": "/pico_left_controller/pose",
    "right_pose": "/pico_right_controller/pose",
    "left_grip": "/pico_left_controller/grip",
    "right_grip": "/pico_right_controller/grip",
    "left_trigger": "/pico_left_controller/trigger",
    "right_trigger": "/pico_right_controller/trigger",
    "left_rate": "/pico_left_controller/rate",
    "right_rate": "/pico_right_controller/rate",
    "right_button_b": "/pico_right_controller/button_b",
    "left_button_x": "/pico_left_controller/button_x",
    "left_button_y": "/pico_left_controller/button_y",
    "head_pose": "/vr/head/pose",
    "pose_mode": "/vr/controller_pose_mode",
}


@dataclass(frozen=True)
class XRobotToolkitSample:
    left_pose_xyzw: tuple[float, ...]
    right_pose_xyzw: tuple[float, ...]
    headset_pose_xyzw: tuple[float, ...]
    left_grip: float
    right_grip: float
    left_trigger: float
    right_trigger: float
    right_button_b: bool
    left_button_x: bool
    left_button_y: bool
    source_timestamp_ns: int


@dataclass(frozen=True)
class XRobotToolkitEventSample:
    right_button_b: bool
    left_button_x: bool
    left_button_y: bool
    source_timestamp_ns: int


class _ClutchInterlock:
    def __init__(self, *, release_stable_s: float):
        self.release_stable_s = float(release_stable_s)
        self._state = "disarmed"
        self._release_since_s: float | None = None

    def fault(self) -> None:
        self._state = "disarmed"
        self._release_since_s = None

    def update(self, pressed: bool, *, now_s: float) -> bool:
        if not pressed:
            if self._release_since_s is None:
                self._release_since_s = now_s
            if now_s - self._release_since_s >= self.release_stable_s:
                self._state = "armed"
            elif self._state == "engaged":
                self._state = "disarmed"
            return False
        self._release_since_s = None
        if self._state == "armed":
            self._state = "engaged"
        return self._state == "engaged"


def _xrobotoolkit_worker(
    *,
    name: str,
    publish_hz: float,
    input_gap_timeout_s: float,
    reconnect_after_s: float,
    release_stable_s: float,
    grip_threshold: float,
    sdk_library_path: str,
    native_log_path: str,
    poses_topic: str,
    triggers_topic: str,
    left_clutch_topic: str,
    right_clutch_topic: str,
    frame_id: str,
    events_only: bool,
    openarmx_compatibility: bool,
    openarmx_head_yaw_normalization: bool,
    openarmx_pose_mode: str,
    openarmx_rate: float,
    openarmx_topics: dict[str, str],
    ros_domain_id: int | None,
    ready_queue: Any,
    event_queue: Any,
) -> None:
    native_log = None
    if native_log_path:
        log_path = Path(native_log_path)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        native_log = log_path.open("ab", buffering=0)
        os.dup2(native_log.fileno(), 1)
        os.dup2(native_log.fileno(), 2)

    import ctypes

    ctypes.CDLL(sdk_library_path, mode=ctypes.RTLD_GLOBAL)
    import rclpy
    import xrobotoolkit_sdk as xrt
    from geometry_msgs.msg import Pose, PoseArray, PoseStamped
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, Float32, String

    stopped = False

    def _stop(_signum: int, _frame: Any) -> None:
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    rclpy.init(
        args=None,
        domain_id=ros_domain_id,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = rclpy.create_node(_safe_node_name(f"nerox_{name}"))
    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )
    poses_pub = node.create_publisher(PoseArray, poses_topic, qos)
    triggers_pub = node.create_publisher(JointState, triggers_topic, qos)
    left_clutch_pub = node.create_publisher(Bool, left_clutch_topic, qos)
    right_clutch_pub = node.create_publisher(Bool, right_clutch_topic, qos)
    openarmx_publishers: dict[str, Any] = {}
    if openarmx_compatibility:
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        message_types = {
            "left_pose": PoseStamped,
            "right_pose": PoseStamped,
            "left_grip": Float32,
            "right_grip": Float32,
            "left_trigger": Float32,
            "right_trigger": Float32,
            "left_rate": Float32,
            "right_rate": Float32,
            "right_button_b": Bool,
            "left_button_x": Bool,
            "left_button_y": Bool,
            "head_pose": PoseStamped,
            "pose_mode": String,
        }
        openarmx_publishers = {
            name: node.create_publisher(
                message_types[name], openarmx_topics[name], reliable_qos
            )
            for name in DEFAULT_OPENARMX_TOPICS
        }

    left_gate = _ClutchInterlock(release_stable_s=release_stable_s)
    right_gate = _ClutchInterlock(release_stable_s=release_stable_s)
    latest: (
        tuple[
            XRobotToolkitSample,
            XRobotToolkitSample,
            bool,
            bool,
            int,
            int,
            int,
        ]
        | None
    ) = None
    last_source_timestamp_ns: int | None = None
    last_source_advance_s = time.monotonic()
    fault_latched = False
    fault_since_s: float | None = None
    fault_publish_pending = False
    next_reconnect_s = time.monotonic() + reconnect_after_s
    ready_reported = False
    sequence = 0
    sdk_initialized = False
    period_s = 1.0 / publish_hz

    try:
        xrt.init()
        sdk_initialized = True
        event_queue.put(
            f"sdk initialized native=1 publish_hz={publish_hz} "
            f"gap_timeout_s={input_gap_timeout_s} "
            f"events_only={int(events_only)} "
            f"openarmx_head_yaw_normalization="
            f"{int(openarmx_head_yaw_normalization)}"
        )
        while rclpy.ok() and not stopped:
            now_s = time.monotonic()
            new_source_sample = False
            try:
                if events_only:
                    event_sample = _read_xr_event_sample(xrt)
                    source_timestamp_ns = event_sample.source_timestamp_ns
                    new_source_sample = _source_timestamp_advanced(
                        source_timestamp_ns=source_timestamp_ns,
                        last_source_timestamp_ns=last_source_timestamp_ns,
                        stalled_s=now_s - last_source_advance_s,
                        timeout_s=input_gap_timeout_s,
                    )
                    if new_source_sample:
                        last_source_timestamp_ns = source_timestamp_ns
                        last_source_advance_s = now_s
                        sequence += 1
                        _publish_openarmx_events(
                            sample=event_sample,
                            Bool=Bool,
                            publishers=openarmx_publishers,
                        )
                        if fault_latched:
                            event_queue.put("XR event input recovered")
                        fault_latched = False
                        fault_since_s = None
                        next_reconnect_s = now_s + reconnect_after_s
                        if not ready_reported:
                            ready_queue.put(
                                {
                                    "sequence": sequence,
                                    "source_timestamp_ns": source_timestamp_ns,
                                }
                            )
                            ready_reported = True
                            event_queue.put(
                                "native controller events publishing "
                                f"source_timestamp_ns={source_timestamp_ns}"
                            )
                    sample = None
                else:
                    sample = _read_xr_sample(xrt)
                    source_timestamp_ns = sample.source_timestamp_ns
                    new_source_sample = _source_timestamp_advanced(
                        source_timestamp_ns=source_timestamp_ns,
                        last_source_timestamp_ns=last_source_timestamp_ns,
                        stalled_s=now_s - last_source_advance_s,
                        timeout_s=input_gap_timeout_s,
                    )

                if not events_only and new_source_sample:
                    openarmx_sample = sample
                    if openarmx_compatibility and openarmx_head_yaw_normalization:
                        openarmx_sample = _head_yaw_localized_sample(sample)
                    last_source_timestamp_ns = source_timestamp_ns
                    last_source_advance_s = now_s
                    sequence += 1
                    left_authorized = left_gate.update(
                        sample.left_grip > grip_threshold,
                        now_s=now_s,
                    )
                    right_authorized = right_gate.update(
                        sample.right_grip > grip_threshold,
                        now_s=now_s,
                    )
                    latest = (
                        sample,
                        openarmx_sample,
                        left_authorized,
                        right_authorized,
                        time.time_ns(),
                        time.monotonic_ns(),
                        sequence,
                    )
                    if fault_latched:
                        event_queue.put("XR input recovered; release interlock active")
                    fault_latched = False
                    fault_since_s = None
                    next_reconnect_s = now_s + reconnect_after_s
            except Exception as exc:
                if not fault_latched:
                    fault_latched = True
                    fault_since_s = now_s
                    left_gate.fault()
                    right_gate.fault()
                    event_queue.put(
                        f"input_fault {type(exc).__name__}: {exc}; clutches disarmed"
                    )
                    fault_publish_pending = True

            # Startup can also stall on the SDK's zero/default poses. Recovery
            # must not depend on having published a valid sample previously.
            if (
                fault_latched
                and fault_since_s is not None
                and now_s - fault_since_s >= reconnect_after_s
                and now_s >= next_reconnect_s
            ):
                try:
                    if sdk_initialized:
                        xrt.close()
                        sdk_initialized = False
                    xrt.init()
                    sdk_initialized = True
                    last_source_timestamp_ns = None
                    last_source_advance_s = now_s
                    event_queue.put("SDK stream reinitialized after input fault")
                except Exception as exc:
                    event_queue.put(
                        f"SDK reinitialize failed {type(exc).__name__}: {exc}"
                    )
                next_reconnect_s = now_s + reconnect_after_s

            if fault_publish_pending:
                _publish_clutches(
                    Bool,
                    left_clutch_pub,
                    right_clutch_pub,
                    False,
                    False,
                )
                if openarmx_publishers:
                    _publish_openarmx_release(
                        Float32=Float32,
                        Bool=Bool,
                        publishers=openarmx_publishers,
                    )
                fault_publish_pending = False

            # The SDK can return the same source sample across several polls.
            # Publishing it again would create duplicate old header stamps and
            # turn one source pause into a long run of unusable alignment
            # targets. Publish poses/triggers exactly once per source advance.
            if (
                not events_only
                and new_source_sample
                and latest is not None
                and not fault_latched
            ):
                (
                    sample,
                    openarmx_sample,
                    left,
                    right,
                    wall_ns,
                    monotonic_ns,
                    local_sequence,
                ) = latest
                _publish_sample(
                    sample=sample,
                    left_authorized=left,
                    right_authorized=right,
                    wall_ns=wall_ns,
                    monotonic_ns=monotonic_ns,
                    sequence=local_sequence,
                    frame_id=frame_id,
                    Pose=Pose,
                    PoseArray=PoseArray,
                    JointState=JointState,
                    Bool=Bool,
                    poses_pub=poses_pub,
                    triggers_pub=triggers_pub,
                    left_clutch_pub=left_clutch_pub,
                    right_clutch_pub=right_clutch_pub,
                )
                if openarmx_publishers:
                    _publish_openarmx_sample(
                        sample=openarmx_sample,
                        left_authorized=left,
                        right_authorized=right,
                        wall_ns=wall_ns,
                        frame_id=frame_id,
                        pose_mode=openarmx_pose_mode,
                        rate=openarmx_rate,
                        PoseStamped=PoseStamped,
                        Float32=Float32,
                        Bool=Bool,
                        String=String,
                        publishers=openarmx_publishers,
                    )
                if not ready_reported:
                    ready_queue.put(
                        {
                            "sequence": local_sequence,
                            "source_timestamp_ns": sample.source_timestamp_ns,
                        }
                    )
                    ready_reported = True
                    event_queue.put(
                        "valid native controller poses publishing "
                        f"source_timestamp_ns={sample.source_timestamp_ns}"
                    )
            time.sleep(period_s)
    finally:
        left_gate.fault()
        right_gate.fault()
        try:
            _publish_clutches(
                Bool,
                left_clutch_pub,
                right_clutch_pub,
                False,
                False,
            )
            if openarmx_publishers:
                _publish_openarmx_release(
                    Float32=Float32,
                    Bool=Bool,
                    publishers=openarmx_publishers,
                )
        except Exception:
            pass
        if sdk_initialized:
            try:
                xrt.close()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        event_queue.put("stopped")
        if native_log is not None:
            native_log.close()


def _source_timestamp_advanced(
    *,
    source_timestamp_ns: int,
    last_source_timestamp_ns: int | None,
    stalled_s: float,
    timeout_s: float,
) -> bool:
    if last_source_timestamp_ns is None:
        return True
    if source_timestamp_ns < last_source_timestamp_ns:
        raise ValueError(
            "XR source timestamp moved backwards: "
            f"{source_timestamp_ns} < {last_source_timestamp_ns}"
        )
    if source_timestamp_ns > last_source_timestamp_ns:
        return True
    if stalled_s > timeout_s:
        raise TimeoutError(f"XR source timestamp has not advanced for {stalled_s:.3f}s")
    return False


def _read_xr_sample(xrt: Any) -> XRobotToolkitSample:
    left_pose = _pose_xyzw(xrt.get_left_controller_pose(), "left controller pose")
    right_pose = _pose_xyzw(xrt.get_right_controller_pose(), "right controller pose")
    headset_pose = _pose_xyzw(xrt.get_headset_pose(), "headset pose")
    timestamp_ns = int(xrt.get_time_stamp_ns())
    if timestamp_ns <= 0:
        raise ValueError(f"XR source timestamp must be positive, got {timestamp_ns}")
    return XRobotToolkitSample(
        left_pose_xyzw=left_pose,
        right_pose_xyzw=right_pose,
        headset_pose_xyzw=headset_pose,
        left_grip=_bounded_input(xrt.get_left_grip(), "left grip"),
        right_grip=_bounded_input(xrt.get_right_grip(), "right grip"),
        left_trigger=_bounded_input(xrt.get_left_trigger(), "left trigger"),
        right_trigger=_bounded_input(xrt.get_right_trigger(), "right trigger"),
        right_button_b=bool(xrt.get_B_button()),
        left_button_x=bool(xrt.get_X_button()),
        left_button_y=bool(xrt.get_Y_button()),
        source_timestamp_ns=timestamp_ns,
    )


def _read_xr_event_sample(xrt: Any) -> XRobotToolkitEventSample:
    timestamp_ns = int(xrt.get_time_stamp_ns())
    if timestamp_ns <= 0:
        raise ValueError(f"XR source timestamp must be positive, got {timestamp_ns}")
    return XRobotToolkitEventSample(
        right_button_b=bool(xrt.get_B_button()),
        left_button_x=bool(xrt.get_X_button()),
        left_button_y=bool(xrt.get_Y_button()),
        source_timestamp_ns=timestamp_ns,
    )


def _head_yaw_localized_sample(
    sample: XRobotToolkitSample,
) -> XRobotToolkitSample:
    """Express controller poses in the headset's horizontal-yaw frame.

    This matches the established XRoboToolkit path used by the NERO simulator:
    the frame origin follows the headset, while headset pitch and roll are not
    included in the basis used to rotate world-space controller poses.
    """

    headset = _xyzw_pose_transform(
        sample.headset_pose_xyzw,
        "headset pose",
    )
    world_to_local = _headset_yaw_world_to_local(headset)
    left = world_to_local @ _xyzw_pose_transform(
        sample.left_pose_xyzw,
        "left controller pose",
    )
    right = world_to_local @ _xyzw_pose_transform(
        sample.right_pose_xyzw,
        "right controller pose",
    )
    return replace(
        sample,
        left_pose_xyzw=_transform_to_xyzw_pose(left),
        right_pose_xyzw=_transform_to_xyzw_pose(right),
    )


def _xyzw_pose_transform(values: Sequence[float], name: str) -> np.ndarray:
    x, y, z, qx, qy, qz, qw = _pose_xyzw(values, name)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray(
        [
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qz * qw),
                2.0 * (qx * qz + qy * qw),
            ],
            [
                2.0 * (qx * qy + qz * qw),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qx * qw),
            ],
            [
                2.0 * (qx * qz - qy * qw),
                2.0 * (qy * qz + qx * qw),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        ],
        dtype=np.float64,
    )
    result[:3, 3] = (x, y, z)
    return result


def _headset_yaw_world_to_local(headset: np.ndarray) -> np.ndarray:
    transform = np.asarray(headset, dtype=np.float64).reshape(4, 4)
    backward = transform[:3, 2].copy()
    backward[1] = 0.0
    norm = float(np.linalg.norm(backward))
    if norm < 1.0e-6:
        raise ValueError("headset forward axis cannot define a horizontal frame")
    backward /= norm
    up = np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
    right = np.cross(up, backward)
    right /= float(np.linalg.norm(right))
    local_to_world_rotation = np.column_stack((right, up, backward))
    world_to_local = np.eye(4, dtype=np.float64)
    world_to_local[:3, :3] = local_to_world_rotation.T
    world_to_local[:3, 3] = -local_to_world_rotation.T @ transform[:3, 3]
    return world_to_local


def _transform_to_xyzw_pose(transform: np.ndarray) -> tuple[float, ...]:
    matrix = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    qw, qx, qy, qz = _rotation_matrix_to_quaternion_wxyz(matrix[:3, :3])
    return (
        *tuple(float(value) for value in matrix[:3, 3]),
        qx,
        qy,
        qz,
        qw,
    )


def _rotation_matrix_to_quaternion_wxyz(
    rotation: np.ndarray,
) -> tuple[float, float, float, float]:
    matrix = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(matrix)):
        raise ValueError("rotation matrix must be finite")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (matrix[2, 1] - matrix[1, 2]) / scale
        qy = (matrix[0, 2] - matrix[2, 0]) / scale
        qz = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            qw = (matrix[2, 1] - matrix[1, 2]) / scale
            qx = 0.25 * scale
            qy = (matrix[0, 1] + matrix[1, 0]) / scale
            qz = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            qw = (matrix[0, 2] - matrix[2, 0]) / scale
            qx = (matrix[0, 1] + matrix[1, 0]) / scale
            qy = 0.25 * scale
            qz = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            qw = (matrix[1, 0] - matrix[0, 1]) / scale
            qx = (matrix[0, 2] + matrix[2, 0]) / scale
            qy = (matrix[1, 2] + matrix[2, 1]) / scale
            qz = 0.25 * scale
    quaternion = np.asarray([qw, qx, qy, qz], dtype=np.float64)
    quaternion /= float(np.linalg.norm(quaternion))
    if quaternion[0] < 0.0:
        quaternion *= -1.0
    return tuple(float(value) for value in quaternion)  # type: ignore[return-value]


def _publish_openarmx_sample(
    *,
    sample: XRobotToolkitSample,
    left_authorized: bool,
    right_authorized: bool,
    wall_ns: int,
    frame_id: str,
    pose_mode: str,
    rate: float,
    PoseStamped: Any,
    Float32: Any,
    Bool: Any,
    String: Any,
    publishers: dict[str, Any],
) -> None:
    source_frame_id = f"{frame_id};source={sample.source_timestamp_ns}"
    for side, values in (
        ("left", sample.left_pose_xyzw),
        ("right", sample.right_pose_xyzw),
    ):
        message = PoseStamped()
        _set_header(message.header, wall_ns, source_frame_id)
        _set_pose(message.pose, values)
        publishers[f"{side}_pose"].publish(message)

    head = PoseStamped()
    _set_header(head.header, wall_ns, source_frame_id)
    _set_pose(head.pose, sample.headset_pose_xyzw)
    publishers["head_pose"].publish(head)

    scalar_values = {
        "left_grip": 1.0 if left_authorized else 0.0,
        "right_grip": 1.0 if right_authorized else 0.0,
        "left_trigger": sample.left_trigger,
        "right_trigger": sample.right_trigger,
        "left_rate": rate,
        "right_rate": rate,
    }
    for name, value in scalar_values.items():
        message = Float32()
        message.data = float(value)
        publishers[name].publish(message)

    for name, value in (
        ("right_button_b", sample.right_button_b),
        ("left_button_x", sample.left_button_x),
        ("left_button_y", sample.left_button_y),
    ):
        message = Bool()
        message.data = bool(value)
        publishers[name].publish(message)

    mode = String()
    mode.data = str(pose_mode)
    publishers["pose_mode"].publish(mode)


def _publish_openarmx_release(
    *,
    Float32: Any,
    Bool: Any,
    publishers: dict[str, Any],
) -> None:
    for name in ("left_grip", "right_grip"):
        message = Float32()
        message.data = 0.0
        publishers[name].publish(message)
    for name in ("right_button_b", "left_button_x", "left_button_y"):
        message = Bool()
        message.data = False
        publishers[name].publish(message)


def _publish_openarmx_events(
    *,
    sample: XRobotToolkitEventSample,
    Bool: Any,
    publishers: dict[str, Any],
) -> None:
    for name, value in (
        ("right_button_b", sample.right_button_b),
        ("left_button_x", sample.left_button_x),
        ("left_button_y", sample.left_button_y),
    ):
        message = Bool()
        message.data = bool(value)
        publishers[name].publish(message)


def _publish_sample(
    *,
    sample: XRobotToolkitSample,
    left_authorized: bool,
    right_authorized: bool,
    wall_ns: int,
    monotonic_ns: int,
    sequence: int,
    frame_id: str,
    Pose: Any,
    PoseArray: Any,
    JointState: Any,
    Bool: Any,
    poses_pub: Any,
    triggers_pub: Any,
    left_clutch_pub: Any,
    right_clutch_pub: Any,
) -> None:
    trace_frame = (
        f"{frame_id};seq={sequence};source={sample.source_timestamp_ns};"
        f"mono={monotonic_ns}"
    )
    triggers = JointState()
    _set_header(triggers.header, wall_ns, trace_frame)
    triggers.name = ["left_trigger", "right_trigger"]
    triggers.position = [sample.left_trigger, sample.right_trigger]
    triggers_pub.publish(triggers)
    _publish_clutches(
        Bool,
        left_clutch_pub,
        right_clutch_pub,
        left_authorized,
        right_authorized,
    )

    # Publish the pose last as the frame commit marker. A consumer that reads
    # between topic callbacks can only observe newer clutch state with an older
    # pose, so releasing Grip always takes effect before the Cartesian update.
    poses = PoseArray()
    _set_header(poses.header, wall_ns, trace_frame)
    poses.poses = [
        _pose_message(Pose, sample.left_pose_xyzw),
        _pose_message(Pose, sample.right_pose_xyzw),
        _pose_message(Pose, sample.headset_pose_xyzw),
    ]
    poses_pub.publish(poses)


def _publish_clutches(
    message_type: Any,
    left_publisher: Any,
    right_publisher: Any,
    left: bool,
    right: bool,
) -> None:
    left_message = message_type()
    left_message.data = bool(left)
    left_publisher.publish(left_message)
    right_message = message_type()
    right_message.data = bool(right)
    right_publisher.publish(right_message)


def _pose_message(message_type: Any, values: Sequence[float]) -> Any:
    pose = message_type()
    _set_pose(pose, values)
    return pose


def _set_pose(pose: Any, values: Sequence[float]) -> None:
    x, y, z, qx, qy, qz, qw = values
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw


def _pose_xyzw(value: Any, name: str) -> tuple[float, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise ValueError(f"{name} must contain 7 values")
    if len(value) != 7:
        raise ValueError(f"{name} must contain 7 values")
    try:
        pose = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must contain numeric values") from exc
    if not all(math.isfinite(item) for item in pose):
        raise ValueError(f"{name} must contain finite values")
    norm = math.sqrt(sum(item * item for item in pose[3:7]))
    if not 0.5 <= norm <= 1.5:
        raise ValueError(f"{name} quaternion norm is invalid: {norm:.6f}")
    return (*pose[:3], *(item / norm for item in pose[3:7]))


def _bounded_input(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return min(1.0, max(0.0, result))


def _set_header(header: Any, wall_ns: int, frame_id: str) -> None:
    header.stamp.sec = int(wall_ns // 1_000_000_000)
    header.stamp.nanosec = int(wall_ns % 1_000_000_000)
    header.frame_id = str(frame_id)


def _safe_node_name(value: str) -> str:
    result = "".join(character if character.isalnum() else "_" for character in value)
    return result.strip("_") or "nerox_xrobotoolkit"


class _PrintSink:
    """Queue-like sink that prints worker readiness and fault events."""

    def __init__(self, prefix: str):
        self.prefix = prefix

    def put(self, item: Any) -> None:
        print(f"[xrt-bridge] {self.prefix}{item}", flush=True)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish XRoboToolkit controller input as OpenArmX-compatible ROS 2 topics"
    )
    parser.add_argument("--sdk-library-path", default=DEFAULT_SDK_LIBRARY_PATH)
    parser.add_argument("--publish-hz", type=float, default=100.0)
    parser.add_argument("--input-gap-timeout-s", type=float, default=0.30)
    parser.add_argument("--reconnect-after-s", type=float, default=0.5)
    parser.add_argument("--release-stable-s", type=float, default=0.25)
    parser.add_argument("--grip-threshold", type=float, default=0.9)
    parser.add_argument(
        "--pose-mode",
        choices=("gripper", "hand"),
        default="gripper",
        help="must match --controller-pose-mode of the teleop node",
    )
    parser.add_argument(
        "--no-head-yaw-normalization",
        action="store_true",
        help="publish raw tracking-space controller poses instead of the Head yaw frame",
    )
    parser.add_argument("--frame-id", default="xrobotoolkit_tracking")
    parser.add_argument("--native-log-path", default="")
    args = parser.parse_args(argv)
    if not 1.0 <= args.publish_hz <= 250.0:
        parser.error("--publish-hz must be in [1, 250]")
    for name in ("input_gap_timeout_s", "reconnect_after_s", "release_stable_s"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive and finite")
    if args.reconnect_after_s <= args.input_gap_timeout_s:
        parser.error("--reconnect-after-s must exceed --input-gap-timeout-s")
    if not 0.0 <= args.grip_threshold <= 1.0:
        parser.error("--grip-threshold must be in [0, 1]")
    if not Path(args.sdk_library_path).expanduser().is_file():
        parser.error(f"XRoboToolkit runtime library not found: {args.sdk_library_path}")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    _xrobotoolkit_worker(
        name="xrobotoolkit",
        publish_hz=args.publish_hz,
        input_gap_timeout_s=args.input_gap_timeout_s,
        reconnect_after_s=args.reconnect_after_s,
        release_stable_s=args.release_stable_s,
        grip_threshold=args.grip_threshold,
        sdk_library_path=str(Path(args.sdk_library_path).expanduser()),
        native_log_path=args.native_log_path,
        poses_topic=DEFAULT_POSES_TOPIC,
        triggers_topic=DEFAULT_TRIGGERS_TOPIC,
        left_clutch_topic=DEFAULT_LEFT_CLUTCH_TOPIC,
        right_clutch_topic=DEFAULT_RIGHT_CLUTCH_TOPIC,
        frame_id=args.frame_id,
        events_only=False,
        openarmx_compatibility=True,
        openarmx_head_yaw_normalization=not args.no_head_yaw_normalization,
        openarmx_pose_mode=args.pose_mode,
        openarmx_rate=0.1,
        openarmx_topics=dict(DEFAULT_OPENARMX_TOPICS),
        ros_domain_id=(
            int(os.environ["ROS_DOMAIN_ID"]) if os.environ.get("ROS_DOMAIN_ID") else None
        ),
        ready_queue=_PrintSink("ready: "),
        event_queue=_PrintSink(""),
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
