import math

import pytest

from nerox_openarmx_teleop.xrobotoolkit_bridge import (
    XRobotToolkitSample,
    XRobotToolkitEventSample,
    _ClutchInterlock,
    _head_yaw_localized_sample,
    _publish_openarmx_release,
    _publish_openarmx_events,
    _publish_openarmx_sample,
    _source_timestamp_advanced,
)


def _sample(
    *,
    left_pose=(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0),
    right_pose=(4.0, 5.0, 6.0, 0.0, 0.0, 0.0, 1.0),
    headset_pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
):
    return XRobotToolkitSample(
        left_pose_xyzw=left_pose,
        right_pose_xyzw=right_pose,
        headset_pose_xyzw=headset_pose,
        left_grip=0.0,
        right_grip=0.0,
        left_trigger=0.0,
        right_trigger=0.0,
        right_button_b=False,
        left_button_x=False,
        left_button_y=False,
        source_timestamp_ns=1,
    )


def test_head_yaw_localization_uses_head_position_as_frame_origin():
    localized = _head_yaw_localized_sample(
        _sample(
            left_pose=(8.0, 10.0, 12.0, 0.0, 0.0, 0.0, 1.0),
            right_pose=(6.0, 6.0, 6.0, 0.0, 0.0, 0.0, 1.0),
            headset_pose=(7.0, 8.0, 9.0, 0.0, 0.0, 0.0, 1.0),
        )
    )

    assert localized.left_pose_xyzw == pytest.approx(
        (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    )
    assert localized.right_pose_xyzw == pytest.approx(
        (-1.0, -2.0, -3.0, 0.0, 0.0, 0.0, 1.0)
    )


def test_head_yaw_localization_removes_head_yaw_from_controller_pose():
    half_angle = math.pi / 4.0
    head_quaternion = (
        0.0,
        math.sin(half_angle),
        0.0,
        math.cos(half_angle),
    )
    # Controller is one metre along Head-local +Z, with the same yaw as Head.
    controller_pose = (1.0, 0.0, 0.0, *head_quaternion)
    localized = _head_yaw_localized_sample(
        _sample(
            left_pose=controller_pose,
            right_pose=controller_pose,
            headset_pose=(0.0, 0.0, 0.0, *head_quaternion),
        )
    )

    assert localized.left_pose_xyzw == pytest.approx(
        (0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        abs=1.0e-12,
    )


def test_xrobotoolkit_duplicate_source_timestamp_is_not_a_new_sample():
    assert not _source_timestamp_advanced(
        source_timestamp_ns=100,
        last_source_timestamp_ns=100,
        stalled_s=0.2,
        timeout_s=1.0,
    )


def test_xrobotoolkit_duplicate_source_timestamp_faults_after_timeout():
    with pytest.raises(TimeoutError, match="not advanced for 1.001s"):
        _source_timestamp_advanced(
            source_timestamp_ns=100,
            last_source_timestamp_ns=100,
            stalled_s=1.001,
            timeout_s=1.0,
        )


def test_xrobotoolkit_source_timestamp_must_not_move_backwards():
    with pytest.raises(ValueError, match="moved backwards"):
        _source_timestamp_advanced(
            source_timestamp_ns=99,
            last_source_timestamp_ns=100,
            stalled_s=0.01,
            timeout_s=1.0,
        )


def test_xrobottoolkit_fault_requires_release_before_reengaging():
    gate = _ClutchInterlock(release_stable_s=0.25)
    assert gate.update(False, now_s=0.0) is False
    assert gate.update(False, now_s=0.25) is False
    assert gate.update(True, now_s=0.26) is True

    gate.fault()
    assert gate.update(True, now_s=1.0) is False
    assert gate.update(False, now_s=1.1) is False
    assert gate.update(False, now_s=1.35) is False
    assert gate.update(True, now_s=1.36) is True


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Header:
    def __init__(self):
        self.stamp = type("Stamp", (), {"sec": 0, "nanosec": 0})()
        self.frame_id = ""


class _Pose:
    def __init__(self):
        self.position = type("Position", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
        self.orientation = type(
            "Orientation",
            (),
            {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0},
        )()


class _PoseStamped:
    def __init__(self):
        self.header = _Header()
        self.pose = _Pose()


class _Value:
    def __init__(self):
        self.data = None


def test_openarmx_compatibility_publish_revokes_and_rearms_grip():
    names = (
        "left_pose",
        "right_pose",
        "left_grip",
        "right_grip",
        "left_trigger",
        "right_trigger",
        "left_rate",
        "right_rate",
        "right_button_b",
        "left_button_x",
        "left_button_y",
        "head_pose",
        "pose_mode",
    )
    publishers = {name: _Publisher() for name in names}
    sample = XRobotToolkitSample(
        left_pose_xyzw=(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0),
        right_pose_xyzw=(4.0, 5.0, 6.0, 0.0, 0.0, 0.0, 1.0),
        headset_pose_xyzw=(7.0, 8.0, 9.0, 0.0, 0.0, 0.0, 1.0),
        left_grip=1.0,
        right_grip=1.0,
        left_trigger=0.25,
        right_trigger=0.75,
        right_button_b=True,
        left_button_x=True,
        left_button_y=False,
        source_timestamp_ns=123,
    )
    _publish_openarmx_sample(
        sample=sample,
        left_authorized=True,
        right_authorized=False,
        wall_ns=2_000_000_003,
        frame_id="xrobotoolkit_tracking",
        pose_mode="gripper",
        rate=0.1,
        PoseStamped=_PoseStamped,
        Float32=_Value,
        Bool=_Value,
        String=_Value,
        publishers=publishers,
    )

    assert publishers["left_grip"].messages[-1].data == 1.0
    assert publishers["right_grip"].messages[-1].data == 0.0
    assert publishers["left_trigger"].messages[-1].data == 0.25
    assert publishers["right_trigger"].messages[-1].data == 0.75
    assert publishers["right_button_b"].messages[-1].data is True
    assert publishers["left_button_x"].messages[-1].data is True
    assert publishers["pose_mode"].messages[-1].data == "gripper"
    assert publishers["left_pose"].messages[-1].header.stamp.sec == 2
    assert publishers["left_pose"].messages[-1].header.stamp.nanosec == 3
    assert publishers["left_pose"].messages[-1].pose.position.x == 1.0

    _publish_openarmx_release(
        Float32=_Value,
        Bool=_Value,
        publishers=publishers,
    )
    assert publishers["left_grip"].messages[-1].data == 0.0
    assert publishers["right_grip"].messages[-1].data == 0.0
    assert publishers["right_button_b"].messages[-1].data is False
    assert publishers["left_button_x"].messages[-1].data is False


def test_openarmx_event_only_publish_does_not_require_controller_poses():
    publishers = {
        "right_button_b": _Publisher(),
        "left_button_x": _Publisher(),
        "left_button_y": _Publisher(),
    }
    _publish_openarmx_events(
        sample=XRobotToolkitEventSample(
            right_button_b=False,
            left_button_x=True,
            left_button_y=False,
            source_timestamp_ns=123,
        ),
        Bool=_Value,
        publishers=publishers,
    )

    assert publishers["left_button_x"].messages[-1].data is True


@pytest.mark.parametrize("recovery", ["valid", "init_failure", "invalid"])
def test_worker_reconnects_before_first_valid_pose(monkeypatch, recovery):
    import ctypes
    import queue
    import sys
    from dataclasses import replace
    from types import SimpleNamespace

    from nerox_openarmx_teleop import xrobotoolkit_bridge as sensor

    clock = [0.0]
    init_times = []
    closes = []
    published = []
    publishers = {}
    ready = queue.Queue()
    events = queue.Queue()

    def init():
        init_times.append(clock[0])
        if recovery == "init_failure" and len(init_times) == 2:
            raise RuntimeError("temporary init failure")

    def sleep(period):
        clock[0] = round(clock[0] + period, 6)

    def publisher(_type, topic, _qos):
        publishers[topic] = _Publisher()
        return publishers[topic]

    node = SimpleNamespace(create_publisher=publisher, destroy_node=lambda: None)
    ros = SimpleNamespace(
        init=lambda **kwargs: None,
        create_node=lambda name: node,
        ok=lambda: clock[0] < 2.0,
        shutdown=lambda: None,
    )
    sdk = SimpleNamespace(init=init, close=lambda: closes.append(clock[0]))
    monkeypatch.setitem(sys.modules, "rclpy", ros)
    monkeypatch.setitem(sys.modules, "rclpy.qos", SimpleNamespace(
        HistoryPolicy=SimpleNamespace(KEEP_LAST=1),
        ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1, RELIABLE=2),
        QoSProfile=lambda **kwargs: None,
    ))
    monkeypatch.setitem(sys.modules, "rclpy.signals", SimpleNamespace(
        SignalHandlerOptions=SimpleNamespace(NO=0),
    ))
    monkeypatch.setitem(sys.modules, "xrobotoolkit_sdk", sdk)
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", SimpleNamespace(
        Pose=_Pose, PoseArray=_Value, PoseStamped=_PoseStamped,
    ))
    monkeypatch.setitem(sys.modules, "sensor_msgs.msg", SimpleNamespace(JointState=_Value))
    monkeypatch.setitem(sys.modules, "std_msgs.msg", SimpleNamespace(
        Bool=_Value, Float32=_Value, String=_Value,
    ))
    monkeypatch.setattr(ctypes, "CDLL", lambda *args, **kwargs: None)
    monkeypatch.setattr(sensor.signal, "signal", lambda *args: None)
    monkeypatch.setattr(sensor.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(sensor.time, "sleep", sleep)

    def read_sample(_sdk):
        required_inits = 3 if recovery == "init_failure" else 2
        valid = recovery != "invalid" and len(init_times) >= required_inits
        pose = (0, 0, 0, 0, 0, 0, 1 if valid else 0)
        sensor._pose_xyzw(pose, "left controller pose")
        return replace(_sample(), left_grip=1.0, right_grip=1.0,
                       source_timestamp_ns=int(clock[0] * 1e9) + 1)

    monkeypatch.setattr(sensor, "_read_xr_sample", read_sample)
    monkeypatch.setattr(sensor, "_publish_sample", lambda **kwargs: published.append(kwargs))
    sensor._xrobotoolkit_worker(
        name="test", publish_hz=10.0, input_gap_timeout_s=0.3,
        reconnect_after_s=0.5, release_stable_s=0.25, grip_threshold=0.9,
        sdk_library_path="unused", native_log_path="",
        poses_topic="poses", triggers_topic="triggers",
        left_clutch_topic="left", right_clutch_topic="right", frame_id="test",
        events_only=False, openarmx_compatibility=False,
        openarmx_head_yaw_normalization=False, openarmx_pose_mode="gripper",
        openarmx_rate=0.1, openarmx_topics={}, ros_domain_id=None,
        ready_queue=ready, event_queue=events,
    )
    assert init_times[:2] == [0.0, 0.5]
    assert all(b - a >= 0.5 for a, b in zip(init_times, init_times[1:]))
    assert all(not msg.data for topic in ("left", "right")
               for msg in publishers[topic].messages)
    if recovery == "invalid":
        assert ready.empty()
        assert published == []
        assert len(init_times) == 4
    else:
        assert ready.qsize() == 1
        assert published
        # Holding Grip through recovery must never authorize motion.
        assert all(not item["left_authorized"] and not item["right_authorized"]
                   for item in published)
        assert len(init_times) == (3 if recovery == "init_failure" else 2)
        if recovery == "init_failure":
            # A failed re-init must not close the already-closed SDK again.
            assert closes == [0.5, 2.0]
