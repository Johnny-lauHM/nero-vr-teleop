from __future__ import annotations

import json
import socket
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

from nerox_openarmx_teleop.tacclaw_remote import (
    PROTOCOL,
    RemoteCalibration,
    TacClawEndpoint,
    TacClawRemoteClient,
    TacClawRemoteError,
    TacClawRemoteWorker,
    TacClawRemoteWorkerState,
    challenge_digest,
    read_token,
)


TOKEN = b"tacclaw-test-token-that-is-longer-than-32-bytes"


def endpoint(*, speed: int = 10) -> TacClawEndpoint:
    return TacClawEndpoint(
        side="left",
        host="192.0.2.10",
        port=55552,
        calibration=RemoteCalibration(closed_raw=269448, open_raw=179448),
        torque=10,
        speed=speed,
        watchdog_s=0.35,
    )


class ScriptedSocket:
    def __init__(self, messages: list[dict]) -> None:
        self.received = bytearray(
            b"".join(
                json.dumps(message, separators=(",", ":")).encode("utf-8")
                + b"\n"
                for message in messages
            )
        )
        self.sent: list[dict] = []
        self.closed = False

    def settimeout(self, _value: float) -> None:
        pass

    def recv(self, size: int) -> bytes:
        if not self.received:
            return b""
        chunk = bytes(self.received[:size])
        del self.received[:size]
        return chunk

    def sendall(self, payload: bytes) -> None:
        self.sent.append(json.loads(payload.decode("utf-8")))

    def shutdown(self, _how: int) -> None:
        pass

    def close(self) -> None:
        self.closed = True


class FakeRemoteClient:
    def __init__(
        self,
        *,
        contact_position: int | None = None,
        follow_commands: bool = True,
        initial_position: int = 1000,
    ) -> None:
        self.endpoint = SimpleNamespace(
            watchdog_s=0.03,
            calibration=endpoint().calibration,
        )
        self.position = initial_position
        self.contact_position = contact_position
        self.follow_commands = follow_commands
        self.calls: list[tuple] = []

    def connect(self) -> dict:
        self.calls.append(("connect",))
        return {"ok": True}

    def status(self) -> dict:
        self.calls.append(("status",))
        return {
            "ok": True,
            "raw_position": self.endpoint.calibration.raw_position(self.position),
            "status": {"error_code": 0},
        }

    def command_position(self, position: int) -> dict:
        self.calls.append(("command", position))
        if self.follow_commands:
            if self.contact_position is not None and position < self.position:
                self.position = max(position, self.contact_position)
            else:
                self.position = position
        return {"ok": True, "accepted_position": position}

    def close(self, *, stop: bool) -> None:
        self.calls.append(("close", stop))


class TacClawRemoteTests(unittest.TestCase):
    def test_speed_50_requires_matching_board_limits(self) -> None:
        for board_speed in (10, 50):
            with self.subTest(board_speed=board_speed):
                scripted = ScriptedSocket(
                    [
                        {"op": "challenge", "protocol": PROTOCOL, "challenge": "nonce"},
                        {
                            "ok": True,
                            "calibration": endpoint().calibration.to_dict(),
                            "limits": {
                                "torque": 10, "speed": board_speed, "watchdog_s": 0.35
                            },
                        },
                    ]
                )
                client = TacClawRemoteClient(endpoint(speed=50), token=TOKEN)
                with mock.patch("socket.create_connection", return_value=scripted):
                    if board_speed == 50:
                        self.assertTrue(client.connect()["ok"])
                        client.close(stop=False)
                    else:
                        with self.assertRaisesRegex(TacClawRemoteError, "safety-limit mismatch"):
                            client.connect()
                self.assertTrue(scripted.closed)
                self.assertEqual([message["op"] for message in scripted.sent], ["authenticate"])

    def test_speed_range_changes_do_not_raise_the_torque_limit(self) -> None:
        for speed in (9, 51, 50.5, True):
            with self.subTest(speed=speed), self.assertRaisesRegex(ValueError, "speed"):
                endpoint(speed=speed)
        with self.assertRaisesRegex(ValueError, "torque"):
            TacClawEndpoint(
                side="left", host="192.0.2.10", port=55552,
                calibration=endpoint().calibration, speed=50, torque=21,
            )

    def test_calibration_maps_normalized_positions(self) -> None:
        calibration = endpoint().calibration
        self.assertEqual(calibration.raw_position(0), 269448)
        self.assertEqual(calibration.raw_position(1000), 179448)
        self.assertEqual(calibration.normalized_position(269448), 0)
        self.assertEqual(calibration.normalized_position(179448), 1000)

    def test_token_permissions_and_length_are_checked(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tacclaw.token"
            path.write_bytes(TOKEN + b"\n")
            path.chmod(0o644)
            with self.assertRaisesRegex(TacClawRemoteError, "permissions"):
                read_token(path)
            path.chmod(0o600)
            self.assertEqual(read_token(path), TOKEN)

    def test_client_authenticates_and_reads_without_motion(self) -> None:
        challenge = "a1b2c3"
        scripted = ScriptedSocket(
            [
                {
                    "op": "challenge",
                    "protocol": PROTOCOL,
                    "challenge": challenge,
                },
                {
                    "ok": True,
                    "calibration": endpoint().calibration.to_dict(),
                    "limits": {"torque": 10, "speed": 10, "watchdog_s": 0.35},
                },
                {
                    "id": 1,
                    "ok": True,
                    "raw_position": 179448,
                    "status": {"error_code": 0},
                },
            ]
        )
        with mock.patch.object(socket, "create_connection", return_value=scripted):
            client = TacClawRemoteClient(endpoint(), token=TOKEN)
            client.connect()
            status = client.status()
            client.close(stop=False)
        self.assertTrue(status["ok"])
        self.assertEqual(
            scripted.sent,
            [
                {
                    "op": "authenticate",
                    "protocol": PROTOCOL,
                    "digest": challenge_digest(TOKEN, challenge),
                },
                {"id": 1, "op": "status"},
            ],
        )
        self.assertNotIn("command", {item.get("op") for item in scripted.sent})

    def test_worker_initializes_and_executes_latest_position(self) -> None:
        client = FakeRemoteClient(initial_position=600)
        worker = TacClawRemoteWorker(
            client,
            name="left",
            command_rate_hz=100.0,
            tracking_timeout_s=0.2,
        )
        worker.start(clearance_confirmed=True)
        deadline = time.monotonic() + 1.0
        while not worker.ready and time.monotonic() < deadline:
            time.sleep(0.001)
        self.assertTrue(worker.ready, worker.error)
        worker.command_position(500, clearance_confirmed=True)
        deadline = time.monotonic() + 1.0
        while worker.last_commanded_position != 500 and time.monotonic() < deadline:
            time.sleep(0.001)
        self.assertEqual(worker.last_commanded_position, 500)
        worker.close(timeout_s=1.0)
        self.assertEqual(worker.state, TacClawRemoteWorkerState.CLOSED)
        self.assertIn(("command", 1000), client.calls)
        self.assertIn(("command", 500), client.calls)

    def test_worker_accepts_partial_close_contact_without_fault(self) -> None:
        client = FakeRemoteClient(contact_position=450)
        worker = TacClawRemoteWorker(
            client,
            name="left",
            command_rate_hz=100.0,
            tracking_timeout_s=0.15,
            tracking_tolerance=10,
            closing_contact_settle_s=0.02,
        )
        worker.start(clearance_confirmed=True)
        deadline = time.monotonic() + 1.0
        while not worker.ready and time.monotonic() < deadline:
            time.sleep(0.001)
        self.assertTrue(worker.ready, worker.error)

        worker.command_position(300, clearance_confirmed=True)
        deadline = time.monotonic() + 1.0
        while worker.last_commanded_position != 300 and time.monotonic() < deadline:
            time.sleep(0.001)
        self.assertEqual(worker.last_commanded_position, 300)
        deadline = time.monotonic() + 1.0
        while worker.tracking_target is not None and time.monotonic() < deadline:
            time.sleep(0.001)

        self.assertTrue(worker.ready, worker.error)
        self.assertEqual(worker.measured_position, 450)
        self.assertIsNone(worker.tracking_target)
        worker.close(timeout_s=1.0)

    def test_worker_accepts_immediate_close_contact_without_fault(self) -> None:
        client = FakeRemoteClient(follow_commands=False)
        worker = TacClawRemoteWorker(
            client,
            name="left",
            command_rate_hz=100.0,
            tracking_timeout_s=0.05,
            tracking_tolerance=10,
            closing_contact_settle_s=0.01,
        )
        worker.start(clearance_confirmed=True)
        deadline = time.monotonic() + 1.0
        while not worker.ready and time.monotonic() < deadline:
            time.sleep(0.001)
        self.assertTrue(worker.ready, worker.error)

        worker.command_position(300, clearance_confirmed=True)
        deadline = time.monotonic() + 1.0
        while worker.last_commanded_position != 300 and time.monotonic() < deadline:
            time.sleep(0.001)
        deadline = time.monotonic() + 1.0
        while worker.tracking_target is not None and time.monotonic() < deadline:
            time.sleep(0.001)

        self.assertTrue(worker.ready, worker.error)
        self.assertEqual(worker.measured_position, 1000)
        self.assertIsNone(worker.tracking_target)
        worker.close(timeout_s=1.0)

    def test_worker_faults_when_opening_does_not_move(self) -> None:
        client = FakeRemoteClient()
        worker = TacClawRemoteWorker(
            client,
            name="left",
            command_rate_hz=100.0,
            tracking_timeout_s=0.05,
            tracking_tolerance=10,
            closing_contact_settle_s=0.01,
        )
        worker.start(clearance_confirmed=True)
        deadline = time.monotonic() + 1.0
        while not worker.ready and time.monotonic() < deadline:
            time.sleep(0.001)
        self.assertTrue(worker.ready, worker.error)

        worker.command_position(300, clearance_confirmed=True)
        deadline = time.monotonic() + 1.0
        while worker.last_commanded_position != 300 and time.monotonic() < deadline:
            time.sleep(0.001)
        deadline = time.monotonic() + 1.0
        while worker.measured_position != 300 and time.monotonic() < deadline:
            time.sleep(0.001)
        self.assertEqual(worker.measured_position, 300)
        client.follow_commands = False
        worker.command_position(800, clearance_confirmed=True)
        deadline = time.monotonic() + 1.0
        while (
            worker.state is not TacClawRemoteWorkerState.FAULT
            and time.monotonic() < deadline
        ):
            time.sleep(0.001)

        self.assertEqual(worker.state, TacClawRemoteWorkerState.FAULT)
        self.assertIn("tracking timeout", worker.error)
        worker.close(timeout_s=1.0)

    def test_worker_allows_closing_contact_to_creep_past_tracking_deadline(self) -> None:
        client = FakeRemoteClient()
        worker = TacClawRemoteWorker(client, name="left")
        with mock.patch("nerox_openarmx_teleop.tacclaw_remote.time.monotonic", return_value=0.0):
            worker._update_measurement(client.status())
        worker._start_tracking(0, 0.0)

        # The 2026-10-12 grasp was still compressing the object near the
        # five-second deadline, leaving less than 0.5 s to settle.
        for now_s, position in (
            (0.2, 909), (1.0, 567), (1.9, 233), (2.5, 206),
            (3.0, 200), (3.6, 196), (4.1, 190), (4.3, 172),
            (4.6, 162), (5.0, 162), (5.2, 158), (5.8, 158),
        ):
            client.position = position
            with mock.patch("nerox_openarmx_teleop.tacclaw_remote.time.monotonic", return_value=now_s):
                worker._update_measurement(client.status())
            worker._check_tracking(now_s)

        self.assertIsNone(worker.tracking_target)
        self.assertEqual(worker.measured_position, 158)

        # Releasing the trigger must still track an opening command.
        worker._start_tracking(1000, 6.0)
        with self.assertRaisesRegex(TacClawRemoteError, "tracking timeout"):
            worker._check_tracking(11.0)

    def test_closing_contact_does_not_mask_board_fault(self) -> None:
        client = FakeRemoteClient()
        worker = TacClawRemoteWorker(client, name="left")
        worker._update_measurement(client.status())
        worker._start_tracking(0, 0.0)
        response = client.status()
        response["status"]["error_code"] = 1
        with self.assertRaisesRegex(TacClawRemoteError, "unhealthy status"):
            worker._update_measurement(response)


if __name__ == "__main__":
    unittest.main()
