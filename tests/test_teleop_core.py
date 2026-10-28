from __future__ import annotations

import contextlib
import io
import math
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from nerox_openarmx_teleop.limiter import OpenArmXJointStepLimiter
from nerox_openarmx_teleop.model import (
    OPENARMX_JOINT_NAMES,
    hardware_to_model,
    model_to_hardware,
)
from nerox_openarmx_teleop.node import (
    OPENARMX_IK_FILTER_WEIGHTS,
    QuestHandStream,
    _make_core,
    build_button_motion_target,
    freshest_trigger,
    limit_button_motion_step,
    log_at_level,
    neutral_relative_pose,
    parse_args,
    reset_relative_handoff,
)
from nerox_openarmx_teleop.tacclaw import TacClawPair, trigger_to_position
from nerox_openarmx_teleop.tacclaw_remote import TacClawRemoteWorkerState
from nerox_openarmx_teleop.xr import (
    NERO_TCP_ORIENTATION_AXIS,
    NERO_TCP_TRANSLATION_AXIS,
    RelativePoseTracker,
    _quaternion_to_rotation,
    yaw_adjusted_axis,
)
from tools.build_nerox_openarmx_model import build_model


ROOT = Path(__file__).resolve().parents[1]


class ModelMappingTests(unittest.TestCase):
    def test_quest_bridge_launcher_executes_the_owned_node_directly(self) -> None:
        launcher = (ROOT / "scripts/start_quest_bridge.sh").read_text(
            encoding="utf-8"
        )
        self.assertIn('exec "${BRIDGE_EXECUTABLE}"', launcher)
        self.assertNotIn("exec ros2 run", launcher)

    def test_openarmx_core_uses_eight_frame_joint_filter(self) -> None:
        core, _pose_type, _relative_type, _absolute_type = _make_core(
            ROOT / "models/nerox_openarmx_bimanual.urdf", 1.0
        )
        np.testing.assert_allclose(
            core.ik_solver.q_filter.weights,
            OPENARMX_IK_FILTER_WEIGHTS,
        )
        self.assertEqual(core.ik_solver.q_filter.window_size, 8)
        self.assertEqual(core.ik_solver.q_filter.dim, 14)

    def test_model_builder_changes_names_without_joint_geometry(self) -> None:
        source_path = ROOT / "models/source/nero_dual_tacclaw.urdf"
        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Path(temp_dir) / "adapted.urdf"
            build_model(source_path, output_path)
            source = ET.parse(source_path).getroot()
            adapted = ET.parse(output_path).getroot()

        source_joints = {
            element.get("name"): element for element in source.findall("joint")
        }
        adapted_joints = {
            element.get("name"): element for element in adapted.findall("joint")
        }
        for arm, side in (("arm_a", "right"), ("arm_b", "left")):
            for index in range(1, 8):
                source_joint = source_joints[f"{arm}_joint{index}"]
                adapted_joint = adapted_joints[
                    f"openarmx_{side}_joint{index}"
                ]
                self.assertEqual(
                    source_joint.find("origin").attrib,
                    adapted_joint.find("origin").attrib,
                )
                self.assertEqual(
                    source_joint.find("axis").attrib,
                    adapted_joint.find("axis").attrib,
                )

    def test_hardware_angles_are_direct_openarmx_coordinates(self) -> None:
        arm_a = np.arange(1.0, 8.0)
        arm_b = -np.arange(1.0, 8.0)
        converted = hardware_to_model({"arm_a": arm_a, "arm_b": arm_b})
        np.testing.assert_allclose(converted, np.concatenate([arm_b, arm_a]))

    def test_hardware_model_round_trip(self) -> None:
        hardware = {
            "arm_a": np.asarray([0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7]),
            "arm_b": np.asarray([-0.7, 0.6, -0.5, 0.4, -0.3, 0.2, -0.1]),
        }
        converted = model_to_hardware(hardware_to_model(hardware))
        np.testing.assert_allclose(converted["arm_a"], hardware["arm_a"])
        np.testing.assert_allclose(converted["arm_b"], hardware["arm_b"])

    def test_openarmx_core_solves_a_small_left_motion(self) -> None:
        core, pose_type, relative_type, absolute_type = _make_core(
            ROOT / "models/nerox_openarmx_bimanual.urdf", 1.0
        )
        hardware = {"arm_a": np.zeros(7), "arm_b": np.zeros(7)}
        current = hardware_to_model(hardware)
        core.update_joint_states(OPENARMX_JOINT_NAMES, current)
        tracker = RelativePoseTracker(
            pose_type,
            NERO_TCP_TRANSLATION_AXIS,
            NERO_TCP_ORIENTATION_AXIS,
        )
        left_tcp, _right_tcp = core.ik_solver.get_end_effector_transforms(current)
        robot_rotation = np.asarray(left_tcp, dtype=np.float64)[:3, :3]
        pose = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        edge = relative_type(
            left_pose=tracker.update(
                pose, True, time.monotonic(), robot_rotation
            ),
            left_grip=1.0,
        )
        first = core.step(edge, absolute_type())
        self.assertTrue(first.left_active)
        np.testing.assert_allclose(first.target_q, current, atol=1e-9)

        pose[0] = 0.005
        core.update_joint_states(OPENARMX_JOINT_NAMES, current)
        moved = relative_type(
            left_pose=tracker.update(
                pose, True, time.monotonic(), robot_rotation
            ),
            left_grip=1.0,
        )
        result = core.step(moved, absolute_type())
        self.assertIsNone(result.error)
        self.assertEqual(np.asarray(result.target_q).shape, (14,))
        self.assertGreater(float(np.linalg.norm(result.target_q[:7] - current[:7])), 0.0)
        np.testing.assert_allclose(result.target_q[7:], current[7:], atol=1e-8)
        target_left_tcp, _target_right_tcp = (
            core.ik_solver.get_end_effector_transforms(np.asarray(result.target_q))
        )
        expected_world_delta = NERO_TCP_TRANSLATION_AXIS @ np.asarray(
            [0.005, 0.0, 0.0]
        )
        np.testing.assert_allclose(
            np.asarray(target_left_tcp)[:3, 3] - np.asarray(left_tcp)[:3, 3],
            expected_world_delta,
            atol=2e-4,
        )

    def test_button_to_grip_handoff_starts_at_measured_feedback(self) -> None:
        core, pose_type, relative_type, absolute_type = _make_core(
            ROOT / "models/nerox_openarmx_bimanual.urdf", 1.0
        )
        before_button = np.zeros(14, dtype=np.float64)
        core.update_joint_states(OPENARMX_JOINT_NAMES, before_button)
        left_tcp, _right_tcp = core.ik_solver.get_end_effector_transforms(
            before_button
        )
        tracker = RelativePoseTracker(
            pose_type,
            NERO_TCP_TRANSLATION_AXIS,
            NERO_TCP_ORIENTATION_AXIS,
        )
        pose = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        core.step(
            relative_type(
                left_pose=tracker.update(
                    pose,
                    True,
                    time.monotonic(),
                    np.asarray(left_tcp)[:3, :3],
                ),
                left_grip=1.0,
            ),
            absolute_type(),
        )
        pose[0] = 0.05
        for _index in range(8):
            core.update_joint_states(OPENARMX_JOINT_NAMES, before_button)
            core.step(
                relative_type(
                    left_pose=tracker.update(
                        pose,
                        True,
                        time.monotonic(),
                        np.asarray(left_tcp)[:3, :3],
                    ),
                    left_grip=1.0,
                ),
                absolute_type(),
            )
        self.assertEqual(len(core.ik_solver.q_filter.buffer), 8)

        button_feedback = np.zeros(14, dtype=np.float64)
        button_feedback[[3, 10]] = 1.5
        reset_relative_handoff(
            core,
            pose_type,
            relative_type,
            absolute_type,
            button_feedback,
            time.monotonic(),
        )
        self.assertFalse(core.relative_left_enabled)
        self.assertFalse(core.relative_right_enabled)
        self.assertEqual(len(core.ik_solver.q_filter.buffer), 0)
        np.testing.assert_allclose(core.ik_solver.q_last, button_feedback)

        tracker.reset()
        left_tcp, _right_tcp = core.ik_solver.get_end_effector_transforms(
            button_feedback
        )
        pose[:] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        first = core.step(
            relative_type(
                left_pose=tracker.update(
                    pose,
                    True,
                    time.monotonic(),
                    np.asarray(left_tcp)[:3, :3],
                ),
                left_grip=1.0,
            ),
            absolute_type(),
        )
        np.testing.assert_allclose(first.target_q, button_feedback, atol=1e-9)

    def test_neutral_relative_pose_releases_enabled_core_side(self) -> None:
        core, pose_type, relative_type, absolute_type = _make_core(
            ROOT / "models/nerox_openarmx_bimanual.urdf", 1.0
        )
        current = np.zeros(14, dtype=np.float64)
        core.update_joint_states(OPENARMX_JOINT_NAMES, current)
        pose = neutral_relative_pose(pose_type, time.monotonic())
        core.step(
            relative_type(left_pose=pose, left_grip=1.0),
            absolute_type(),
        )
        self.assertTrue(core.relative_left_enabled)

        core.step(
            relative_type(left_pose=pose, left_grip=0.0),
            absolute_type(),
        )
        self.assertFalse(core.relative_left_enabled)


class LimiterTests(unittest.TestCase):
    def test_openarmx_joint_step_profile_is_retained(self) -> None:
        limiter = OpenArmXJointStepLimiter()
        command = limiter.limit(np.full(7, math.pi), np.zeros(7))
        np.testing.assert_allclose(command, np.radians(np.full(7, 4.0)))

    def test_openarmx_fast_joint_step_profile_is_retained(self) -> None:
        limiter = OpenArmXJointStepLimiter()
        command = limiter.limit(np.full(7, math.pi), np.zeros(7), fast_mode=True)
        np.testing.assert_allclose(
            command,
            np.radians(np.asarray([20.0, 16.0, 12.0, 12.0, 14.0, 8.0, 12.0])),
        )

    def test_openarmx_limit_releases_below_enable_threshold(self) -> None:
        limiter = OpenArmXJointStepLimiter()
        target = np.radians(np.full(7, 3.0))
        np.testing.assert_allclose(limiter.limit(target, np.zeros(7)), target)

    def test_official_button_targets_and_step(self) -> None:
        upper = np.full(14, 3.0)
        home = build_button_motion_target("go_home", upper)
        np.testing.assert_allclose(home, np.zeros(14))
        home_hardware = model_to_hardware(home)
        np.testing.assert_allclose(home_hardware["arm_a"], np.zeros(7))
        np.testing.assert_allclose(home_hardware["arm_b"], np.zeros(7))
        hands_up = build_button_motion_target("hands_up", upper)
        expected = np.zeros(14)
        expected[[3, 10]] = math.pi / 2.0
        np.testing.assert_allclose(hands_up, expected)
        stepped = limit_button_motion_step(
            hands_up, np.zeros(14), math.radians(0.5)
        )
        expected_step = np.zeros(14)
        expected_step[[3, 10]] = math.radians(0.5)
        np.testing.assert_allclose(stepped, expected_step)


class QuestTcpAxisTests(unittest.TestCase):
    def test_relative_tracker_exposes_position_diagnostics(self) -> None:
        tracker = RelativePoseTracker(
            SimpleNamespace,
            NERO_TCP_TRANSLATION_AXIS,
            NERO_TCP_ORIENTATION_AXIS,
        )
        reference = np.asarray([0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0])
        tracker.update(reference, True, 0.0, np.eye(3))
        current = reference.copy()
        current[:3] += np.asarray([0.01, -0.02, 0.03])
        tracker.update(current, True, 0.1, np.eye(3))

        diagnostics = tracker.position_diagnostics()
        self.assertIsNotNone(diagnostics)
        anchor, raw_pose, raw_delta, mapped_delta = diagnostics
        np.testing.assert_allclose(anchor, reference)
        np.testing.assert_allclose(raw_pose, current)
        np.testing.assert_allclose(raw_delta, current[:3] - reference[:3])
        np.testing.assert_allclose(
            mapped_delta,
            NERO_TCP_TRANSLATION_AXIS @ raw_delta,
        )

        tracker.reset()
        self.assertIsNone(tracker.position_diagnostics())

    def test_180_degree_input_yaw_flips_both_horizontal_tcp_axes(self) -> None:
        adjusted = yaw_adjusted_axis(NERO_TCP_TRANSLATION_AXIS, 180.0)
        np.testing.assert_allclose(
            adjusted,
            np.asarray(
                [
                    [0.0, 0.0, 1.0],
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                ]
            ),
            atol=1e-15,
        )

    def test_input_yaw_cli_is_explicit_and_bounded(self) -> None:
        args = parse_args(["--check", "--input-yaw-deg", "180"])
        self.assertEqual(args.input_yaw_deg, 180.0)
        with (
            contextlib.redirect_stderr(io.StringIO()),
            self.assertRaises(SystemExit),
        ):
            parse_args(["--check", "--input-yaw-deg", "181"])

    def test_nero_tcp_translation_and_rotation_basis(self) -> None:
        tracker = RelativePoseTracker(
            SimpleNamespace,
            NERO_TCP_TRANSLATION_AXIS,
            NERO_TCP_ORIENTATION_AXIS,
        )
        robot_rotation = np.asarray(
            [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]],
            dtype=np.float64,
        )
        reference = np.asarray([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        tracker.update(reference, True, 0.0, robot_rotation)

        angle = 0.2
        current = np.asarray(
            [0.1, 0.2, 0.3, math.sin(angle / 2.0), 0.0, 0.0, math.cos(angle / 2.0)]
        )
        mapped = tracker.update(current, True, 0.1)

        expected_world_position = NERO_TCP_TRANSLATION_AXIS @ current[:3]
        np.testing.assert_allclose(
            robot_rotation @ mapped.position, expected_world_position
        )
        input_rotation = _quaternion_to_rotation(current)
        expected_rotation = (
            NERO_TCP_ORIENTATION_AXIS
            @ input_rotation
            @ NERO_TCP_ORIENTATION_AXIS.T
        )
        mapped_pose = np.concatenate(
            [np.zeros(3), np.asarray(mapped.orientation_xyzw, dtype=np.float64)]
        )
        mapped_local_rotation = _quaternion_to_rotation(mapped_pose)
        np.testing.assert_allclose(
            robot_rotation @ mapped_local_rotation @ robot_rotation.T,
            expected_rotation,
            atol=1e-12,
        )


class QuestTacClawPolicyTests(unittest.TestCase):
    def test_ros_logger_severity_uses_distinct_call_sites(self) -> None:
        calls = []
        logger = SimpleNamespace(
            error=lambda message: calls.append(("error", message)),
            warning=lambda message: calls.append(("warning", message)),
            info=lambda message: calls.append(("info", message)),
        )
        log_at_level(logger, "warn", "connecting")
        log_at_level(logger, "error", "fault")
        self.assertEqual(
            calls,
            [("warning", "connecting"), ("error", "fault")],
        )

    def test_pair_snapshot_exposes_validity_without_board_connection(self) -> None:
        pair = TacClawPair.__new__(TacClawPair)
        pair.workers = {}
        for side, position, state in (
            ("left", 700, TacClawRemoteWorkerState.READY),
            ("right", 600, TacClawRemoteWorkerState.FAULT),
        ):
            worker = SimpleNamespace(
                condition=contextlib.nullcontext(),
                _state=state,
                _error="timeout" if side == "right" else "",
                measured_position=position,
                last_commanded_position=position - 10,
            )
            pair.workers[side] = worker
        snapshot = pair.snapshot()
        self.assertTrue(snapshot["left"]["ready"])
        self.assertFalse(snapshot["right"]["ready"])
        self.assertEqual(snapshot["right"]["measured_position"], 600)

    def test_trigger_is_selected_without_grip(self) -> None:
        relative = QuestHandStream(
            grip=0.0,
            trigger=0.75,
            trigger_received_s=10.0,
        )
        absolute = QuestHandStream(
            grip=0.0,
            trigger=0.25,
            trigger_received_s=9.8,
        )
        self.assertEqual(
            freshest_trigger((relative, absolute), 10.1, 0.3),
            0.75,
        )

    def test_quest_trigger_maps_open_to_closed(self) -> None:
        self.assertEqual(trigger_to_position(0.0), 1000)
        self.assertEqual(trigger_to_position(0.5), 500)
        self.assertEqual(trigger_to_position(1.0), 0)

    def test_tacclaw_default_torque_limit_is_twenty(self) -> None:
        args = parse_args(["--check"])
        self.assertEqual(args.tacclaw_torque_limit, 20)

    def test_tacclaw_speed_fifty_preserves_torque_limit(self) -> None:
        args = parse_args(["--check", "--tacclaw-speed", "50"])
        self.assertEqual(args.tacclaw_speed, 50)
        self.assertEqual(args.tacclaw_torque_limit, 20)
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                parse_args(["--check", "--tacclaw-speed", "51"])
            with self.assertRaises(SystemExit):
                parse_args(["--check", "--tacclaw-speed", "50", "--tacclaw-torque-limit", "21"])

    def test_tacclaw_requires_execute_and_clearance(self) -> None:
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                parse_args(["--shadow", "--with-tacclaw"])
            with self.assertRaises(SystemExit):
                parse_args(["--execute", "--with-tacclaw"])


if __name__ == "__main__":
    unittest.main()
