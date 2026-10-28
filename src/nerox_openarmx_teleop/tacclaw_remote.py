from __future__ import annotations

import hashlib
import hmac
import json
import math
import socket
import threading
import time
from collections.abc import Mapping
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any


PROTOCOL = "teleop-tacclaw-can-v1"
MAX_MESSAGE_BYTES = 4096


class TacClawRemoteError(RuntimeError):
    pass


@dataclass(frozen=True)
class RemoteCalibration:
    closed_raw: int
    open_raw: int
    speed_coefficient: int = 3600

    def __post_init__(self) -> None:
        if isinstance(self.closed_raw, bool) or not isinstance(self.closed_raw, int):
            raise ValueError("closed_raw must be an integer")
        if isinstance(self.open_raw, bool) or not isinstance(self.open_raw, int):
            raise ValueError("open_raw must be an integer")
        if self.travel_raw != 90_000:
            raise ValueError("validated TacClaw travel must be exactly 90000 counts")
        if self.speed_coefficient != 3600:
            raise ValueError("TacClaw speed coefficient must be 3600")

    @property
    def travel_raw(self) -> int:
        return self.closed_raw - self.open_raw

    def to_dict(self) -> dict[str, int]:
        return {
            "closed_raw": self.closed_raw,
            "open_raw": self.open_raw,
            "speed_coefficient": self.speed_coefficient,
            "travel_raw": self.travel_raw,
        }

    def raw_position(self, position: int) -> int:
        resolved = _position(position)
        return round(
            self.closed_raw - (resolved / 1000.0) * self.travel_raw
        )

    def normalized_position(self, raw_position: int) -> int:
        if isinstance(raw_position, bool) or not isinstance(raw_position, int):
            raise ValueError("raw_position must be an integer")
        value = round(
            1000.0 * (self.closed_raw - raw_position) / self.travel_raw
        )
        return min(1000, max(0, value))

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "RemoteCalibration":
        try:
            return cls(
                closed_raw=value["closed_raw"],
                open_raw=value["open_raw"],
                speed_coefficient=value["speed_coefficient"],
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise TacClawRemoteError(
                f"invalid remote calibration: {value}"
            ) from exc


@dataclass(frozen=True)
class TacClawEndpoint:
    side: str
    host: str
    port: int
    calibration: RemoteCalibration
    torque: int = 10
    speed: int = 10
    watchdog_s: float = 0.35

    def __post_init__(self) -> None:
        if self.side not in {"left", "right"}:
            raise ValueError("side must be left or right")
        if not self.host.strip():
            raise ValueError("host must not be empty")
        if not 1 <= self.port <= 65535:
            raise ValueError("port must be in 1..65535")
        if not 10 <= self.torque <= 20:
            raise ValueError("torque must be in 10..20")
        if type(self.speed) is not int or not 10 <= self.speed <= 50:
            raise ValueError("speed must be an integer in 10..50")
        if not math.isfinite(self.watchdog_s) or not 0.2 <= self.watchdog_s <= 1.0:
            raise ValueError("watchdog_s must be in 0.2..1.0")


def read_token(path: str | Path) -> bytes:
    token_path = Path(path).expanduser().resolve()
    try:
        mode = token_path.stat().st_mode
        token = token_path.read_bytes().strip()
    except OSError as exc:
        raise TacClawRemoteError(
            f"cannot read TacClaw token {token_path}: {exc}"
        ) from exc
    if mode & 0o077:
        raise TacClawRemoteError(
            f"TacClaw token permissions must deny group/other access: {token_path}"
        )
    if len(token) < 32:
        raise TacClawRemoteError("TacClaw token must contain at least 32 bytes")
    return token


def challenge_digest(token: bytes, challenge: str) -> str:
    if len(token) < 32:
        raise ValueError("token must contain at least 32 bytes")
    if not challenge or len(challenge) > 256:
        raise ValueError("challenge has an invalid length")
    try:
        encoded = challenge.encode("ascii")
    except UnicodeEncodeError as exc:
        raise ValueError("challenge must be ASCII") from exc
    return hmac.new(token, encoded, hashlib.sha256).hexdigest()


class _JsonLineConnection:
    def __init__(self, sock: socket.socket) -> None:
        self.sock = sock
        self.buffer = bytearray()

    def send(self, message: Mapping[str, Any]) -> None:
        try:
            payload = json.dumps(
                dict(message),
                separators=(",", ":"),
                sort_keys=True,
                allow_nan=False,
            ).encode("utf-8")
        except (TypeError, ValueError) as exc:
            raise TacClawRemoteError(f"cannot encode protocol message: {exc}") from exc
        if len(payload) > MAX_MESSAGE_BYTES:
            raise TacClawRemoteError("protocol message exceeds size limit")
        self.sock.sendall(payload + b"\n")

    def receive(self) -> dict[str, Any]:
        while b"\n" not in self.buffer:
            chunk = self.sock.recv(1024)
            if not chunk:
                raise TacClawRemoteError("TacClaw service closed the connection")
            self.buffer.extend(chunk)
            if len(self.buffer) > MAX_MESSAGE_BYTES:
                raise TacClawRemoteError("protocol message exceeds size limit")
        payload, _, remaining = self.buffer.partition(b"\n")
        self.buffer = bytearray(remaining)
        try:
            result = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise TacClawRemoteError("received malformed protocol JSON") from exc
        if not isinstance(result, dict):
            raise TacClawRemoteError("protocol message must be a JSON object")
        return result


class TacClawRemoteClient:
    def __init__(
        self,
        endpoint: TacClawEndpoint,
        *,
        token: bytes,
        connect_timeout_s: float = 1.0,
        request_timeout_s: float = 0.75,
    ) -> None:
        if len(token) < 32:
            raise ValueError("token must contain at least 32 bytes")
        for value, name in (
            (connect_timeout_s, "connect_timeout_s"),
            (request_timeout_s, "request_timeout_s"),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        self.endpoint = endpoint
        self.token = bytes(token)
        self.connect_timeout_s = float(connect_timeout_s)
        self.request_timeout_s = float(request_timeout_s)
        self.sock: socket.socket | None = None
        self.connection: _JsonLineConnection | None = None
        self.request_id = 0

    def connect(self) -> dict[str, Any]:
        if self.connection is not None:
            raise TacClawRemoteError(
                f"{self.endpoint.side} TacClaw is already connected"
            )
        sock = socket.create_connection(
            (self.endpoint.host, self.endpoint.port),
            timeout=self.connect_timeout_s,
        )
        sock.settimeout(self.request_timeout_s)
        connection = _JsonLineConnection(sock)
        try:
            challenge = connection.receive()
            if (
                challenge.get("protocol") != PROTOCOL
                or challenge.get("op") != "challenge"
            ):
                raise TacClawRemoteError("unexpected TacClaw authentication challenge")
            nonce = challenge.get("challenge")
            if not isinstance(nonce, str):
                raise TacClawRemoteError("TacClaw challenge is missing")
            connection.send(
                {
                    "op": "authenticate",
                    "protocol": PROTOCOL,
                    "digest": challenge_digest(self.token, nonce),
                }
            )
            authenticated = connection.receive()
            if authenticated.get("ok") is not True:
                raise TacClawRemoteError(
                    f"TacClaw authentication failed: {authenticated.get('error', '')}"
                )
            actual_calibration = RemoteCalibration.from_mapping(
                _mapping(authenticated.get("calibration"), "calibration")
            )
            if actual_calibration != self.endpoint.calibration:
                raise TacClawRemoteError(
                    f"{self.endpoint.side} calibration mismatch: "
                    f"expected={self.endpoint.calibration.to_dict()} "
                    f"actual={actual_calibration.to_dict()}"
                )
            limits = _mapping(authenticated.get("limits"), "limits")
            actual_limits = {
                "torque": int(limits["torque"]),
                "speed": int(limits["speed"]),
                "watchdog_s": float(limits["watchdog_s"]),
            }
            expected_limits = {
                "torque": self.endpoint.torque,
                "speed": self.endpoint.speed,
                "watchdog_s": self.endpoint.watchdog_s,
            }
            if actual_limits != expected_limits:
                raise TacClawRemoteError(
                    f"{self.endpoint.side} safety-limit mismatch: "
                    f"expected={expected_limits} actual={actual_limits}"
                )
        except Exception:
            sock.close()
            raise
        self.sock = sock
        self.connection = connection
        self.request_id = 0
        return authenticated

    def _request(self, operation: str, **values: Any) -> dict[str, Any]:
        if self.connection is None:
            raise TacClawRemoteError(f"{self.endpoint.side} TacClaw is not connected")
        self.request_id += 1
        request_id = self.request_id
        self.connection.send({"id": request_id, "op": operation, **values})
        response = self.connection.receive()
        if response.get("id") != request_id:
            raise TacClawRemoteError("TacClaw response id mismatch")
        if response.get("ok") is not True:
            raise TacClawRemoteError(
                f"{self.endpoint.side} TacClaw {operation} failed: "
                f"{response.get('error', 'unknown error')}"
            )
        return response

    def status(self) -> dict[str, Any]:
        return self._request("status")

    def command_position(self, position: int) -> dict[str, Any]:
        return self._request("command", position=_position(position))

    def stop(self) -> dict[str, Any]:
        return self._request("stop")

    def close(self, *, stop: bool = True) -> None:
        connection = self.connection
        sock = self.sock
        if connection is not None and stop:
            try:
                self.stop()
            except (OSError, TacClawRemoteError, socket.timeout):
                pass
        self.connection = None
        self.sock = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()


class TacClawRemoteWorkerState(str, Enum):
    DISABLED = "disabled"
    CONNECTING = "connecting"
    READY = "ready"
    FAULT = "fault"
    CLOSED = "closed"


class TacClawRemoteWorker:
    def __init__(
        self,
        client: TacClawRemoteClient,
        *,
        name: str,
        command_rate_hz: float = 20.0,
        position_deadband: int = 3,
        initial_position: int = 1000,
        tracking_timeout_s: float = 5.0,
        tracking_tolerance: int = 20,
        closing_contact_settle_s: float = 0.5,
    ) -> None:
        if not name:
            raise ValueError("name must not be empty")
        if not math.isfinite(command_rate_hz) or command_rate_hz <= 0.0:
            raise ValueError("command_rate_hz must be positive and finite")
        if not 0 <= position_deadband <= 1000:
            raise ValueError("position_deadband must be in 0..1000")
        if not math.isfinite(tracking_timeout_s) or tracking_timeout_s <= 0.0:
            raise ValueError("tracking_timeout_s must be positive and finite")
        if not 1 <= tracking_tolerance <= 100:
            raise ValueError("tracking_tolerance must be in 1..100")
        if (
            not math.isfinite(closing_contact_settle_s)
            or closing_contact_settle_s <= 0.0
        ):
            raise ValueError("closing_contact_settle_s must be positive and finite")
        self.client = client
        self.name = name
        self.command_rate_hz = float(command_rate_hz)
        self.position_deadband = int(position_deadband)
        self.initial_position = _position(initial_position)
        self.heartbeat_period_s = client.endpoint.watchdog_s / 3.0
        self.tracking_timeout_s = float(tracking_timeout_s)
        self.tracking_tolerance = int(tracking_tolerance)
        self.closing_contact_settle_s = float(closing_contact_settle_s)
        self.condition = threading.Condition()
        self.thread: threading.Thread | None = None
        self.pending_position: int | None = None
        self.last_accepted_position: int | None = None
        self.last_commanded_position: int | None = None
        self.command_sequence = 0
        self.command_send_wall_ns: int | None = None
        self.command_send_monotonic_ns: int | None = None
        self.command_ack_wall_ns: int | None = None
        self.command_ack_monotonic_ns: int | None = None
        self.measured_position: int | None = None
        self.measured_raw_position: int | None = None
        self.tracking_target: int | None = None
        self.tracking_started_s: float | None = None
        self.tracking_start_position: int | None = None
        self.closing_sequence_start_position: int | None = None
        self.tracking_motion_position: int | None = None
        self.tracking_last_motion_s: float | None = None
        self.remote_status: dict[str, Any] = {}
        self._state = TacClawRemoteWorkerState.DISABLED
        self._error = ""
        self.stopping = False

    @property
    def state(self) -> TacClawRemoteWorkerState:
        with self.condition:
            return self._state

    @property
    def error(self) -> str:
        with self.condition:
            return self._error

    @property
    def ready(self) -> bool:
        return self.state is TacClawRemoteWorkerState.READY

    def start(self, *, clearance_confirmed: bool) -> None:
        if not clearance_confirmed:
            raise TacClawRemoteError("worker start requires clearance confirmation")
        with self.condition:
            if self.thread is not None or self._state is not TacClawRemoteWorkerState.DISABLED:
                raise TacClawRemoteError("worker is already started")
            self.stopping = False
            self._state = TacClawRemoteWorkerState.CONNECTING
            self.thread = threading.Thread(
                target=self._run,
                name=f"tacclaw-remote-{self.name}",
                daemon=True,
            )
            self.thread.start()

    def command_position(self, position: int, *, clearance_confirmed: bool) -> int:
        if not clearance_confirmed:
            raise TacClawRemoteError("motion requires clearance confirmation")
        resolved = _position(position)
        with self.condition:
            if self._state is not TacClawRemoteWorkerState.READY:
                raise TacClawRemoteError(
                    f"worker is not ready: {self._state.value} {self._error}".rstrip()
                )
            reference = self.last_accepted_position
            if reference is not None and abs(resolved - reference) < self.position_deadband:
                return reference
            self.last_accepted_position = resolved
            self.pending_position = resolved
            self.condition.notify_all()
        return resolved

    def cancel_pending(self) -> None:
        with self.condition:
            self.pending_position = None
            self.condition.notify_all()

    def close(self, *, timeout_s: float = 2.0) -> None:
        with self.condition:
            self.stopping = True
            self.pending_position = None
            self.condition.notify_all()
            thread = self.thread
        if thread is not None:
            thread.join(timeout=timeout_s)
            if thread.is_alive():
                raise TacClawRemoteError("worker did not stop before timeout")
        else:
            self.client.close(stop=True)
        with self.condition:
            self.thread = None
            if self._state is not TacClawRemoteWorkerState.FAULT:
                self._state = TacClawRemoteWorkerState.CLOSED

    def _run(self) -> None:
        last_activity_s = 0.0
        next_command_s = 0.0
        try:
            self.client.connect()
            self._update_measurement(self.client.status())
            send_wall_ns = time.time_ns()
            send_monotonic_ns = time.monotonic_ns()
            initial = self.client.command_position(self.initial_position)
            ack_wall_ns = time.time_ns()
            ack_monotonic_ns = time.monotonic_ns()
            accepted = _accepted_position(initial, self.initial_position)
            now_s = time.monotonic()
            last_activity_s = now_s
            next_command_s = now_s + 1.0 / self.command_rate_hz
            self._start_tracking(accepted, now_s)
            while not self._tracking_complete():
                if self._stopping():
                    return
                time.sleep(self.heartbeat_period_s)
                self._update_measurement(self.client.status())
                last_activity_s = time.monotonic()
                self._check_tracking(last_activity_s)
                with self.condition:
                    if self.tracking_target is None:
                        break
            with self.condition:
                self.last_accepted_position = accepted
                self.last_commanded_position = accepted
                self._record_applied_command(
                    send_wall_ns=send_wall_ns,
                    send_monotonic_ns=send_monotonic_ns,
                    ack_wall_ns=ack_wall_ns,
                    ack_monotonic_ns=ack_monotonic_ns,
                )
                self._state = TacClawRemoteWorkerState.READY
                self.condition.notify_all()

            while True:
                with self.condition:
                    if self.stopping:
                        return
                    now_s = time.monotonic()
                    can_command = (
                        self.pending_position is not None and now_s >= next_command_s
                    )
                    heartbeat_due = now_s >= last_activity_s + self.heartbeat_period_s
                    if not can_command and not heartbeat_due:
                        wait_s = last_activity_s + self.heartbeat_period_s - now_s
                        if self.pending_position is not None:
                            wait_s = min(wait_s, max(0.0, next_command_s - now_s))
                        self.condition.wait(timeout=wait_s)
                        continue
                    position = self.pending_position if can_command else None
                    if can_command:
                        self.pending_position = None
                if position is not None:
                    send_wall_ns = time.time_ns()
                    send_monotonic_ns = time.monotonic_ns()
                    response = self.client.command_position(position)
                    ack_wall_ns = time.time_ns()
                    ack_monotonic_ns = time.monotonic_ns()
                    accepted = _accepted_position(response, position)
                    now_s = time.monotonic()
                    self._start_tracking(accepted, now_s)
                    next_command_s = now_s + 1.0 / self.command_rate_hz
                    with self.condition:
                        self.last_commanded_position = accepted
                        self._record_applied_command(
                            send_wall_ns=send_wall_ns,
                            send_monotonic_ns=send_monotonic_ns,
                            ack_wall_ns=ack_wall_ns,
                            ack_monotonic_ns=ack_monotonic_ns,
                        )
                        self.remote_status = dict(response)
                else:
                    self._update_measurement(self.client.status())
                    self._check_tracking(time.monotonic())
                last_activity_s = time.monotonic()
        except Exception as exc:
            with self.condition:
                self._error = str(exc)
                self._state = TacClawRemoteWorkerState.FAULT
                self.pending_position = None
                self.condition.notify_all()
        finally:
            self.client.close(stop=True)
            with self.condition:
                if self._state is not TacClawRemoteWorkerState.FAULT:
                    self._state = TacClawRemoteWorkerState.CLOSED
                self.condition.notify_all()

    def _stopping(self) -> bool:
        with self.condition:
            return self.stopping

    def _record_applied_command(
        self,
        *,
        send_wall_ns: int,
        send_monotonic_ns: int,
        ack_wall_ns: int,
        ack_monotonic_ns: int,
    ) -> None:
        # Caller holds condition. Record only commands acknowledged by the
        # board service so collection never labels a queued-but-unsent target.
        self.command_sequence += 1
        self.command_send_wall_ns = int(send_wall_ns)
        self.command_send_monotonic_ns = int(send_monotonic_ns)
        self.command_ack_wall_ns = int(ack_wall_ns)
        self.command_ack_monotonic_ns = int(ack_monotonic_ns)

    def _start_tracking(self, target: int, now_s: float) -> None:
        with self.condition:
            resolved = _position(target)
            measured = self.measured_position
            if measured is not None and resolved < measured:
                if self.closing_sequence_start_position is None:
                    self.closing_sequence_start_position = measured
            else:
                self.closing_sequence_start_position = None
            self.tracking_target = resolved
            self.tracking_started_s = now_s
            self.tracking_start_position = measured
            self.tracking_motion_position = measured
            self.tracking_last_motion_s = now_s

    def _update_measurement(self, response: Mapping[str, Any]) -> None:
        if not _healthy_status(response):
            raise TacClawRemoteError(
                f"{self.name} TacClaw reported unhealthy status: {dict(response)}"
            )
        raw = response.get("raw_position")
        if isinstance(raw, bool) or not isinstance(raw, int):
            raise TacClawRemoteError("TacClaw status is missing raw_position")
        measured = self.client.endpoint.calibration.normalized_position(raw)
        now_s = time.monotonic()
        with self.condition:
            previous = self.tracking_motion_position
            if previous is None or abs(measured - previous) > 3:
                self.tracking_motion_position = measured
                self.tracking_last_motion_s = now_s
            self.measured_position = measured
            self.measured_raw_position = raw
            self.remote_status = dict(response)

    def _tracking_error(self) -> int | None:
        with self.condition:
            target = self.tracking_target
            raw = self.measured_raw_position
        if target is None or raw is None:
            return None
        calibration = self.client.endpoint.calibration
        return round(
            1000.0
            * abs(calibration.raw_position(target) - raw)
            / calibration.travel_raw
        )

    def _tracking_complete(self) -> bool:
        error = self._tracking_error()
        return error is not None and error <= self.tracking_tolerance

    def _check_tracking(self, now_s: float) -> None:
        if self._tracking_complete():
            with self.condition:
                self.tracking_target = None
                self.tracking_started_s = None
            return
        with self.condition:
            target = self.tracking_target
            started = self.tracking_started_s
            closing_start = self.closing_sequence_start_position
            measured = self.measured_position
            last_motion = self.tracking_last_motion_s
        if target is None or started is None:
            return
        # Any closing command may stop on a grasped object before reaching its
        # requested position. Position feedback alone cannot distinguish an
        # immediate contact from a jam, so healthy closing motion may keep
        # compressing an object past the position-tracking deadline. Wait for
        # contact to settle; only opening commands require position tracking.
        if (
            closing_start is not None
            and measured is not None
            and target < closing_start
            and target < measured
        ):
            if (
                last_motion is not None
                and now_s - last_motion >= self.closing_contact_settle_s
            ):
                with self.condition:
                    self.tracking_target = None
                    self.tracking_started_s = None
            return
        if now_s - started >= self.tracking_timeout_s:
            raise TacClawRemoteError(
                f"{self.name} TacClaw tracking timeout: target={target} "
                f"measured={measured} error={self._tracking_error()}"
            )


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise TacClawRemoteError(f"{name} must be a mapping")
    return value


def _position(value: int, *, name: str = "position") -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    if not 0 <= value <= 1000:
        raise ValueError(f"{name} must be in 0..1000")
    return value


def _accepted_position(response: Mapping[str, Any], requested: int) -> int:
    value = response.get("accepted_position", requested)
    try:
        return _position(value)
    except ValueError as exc:
        raise TacClawRemoteError(
            f"invalid accepted_position from TacClaw service: {value!r}"
        ) from exc


def _healthy_status(response: Mapping[str, Any]) -> bool:
    if response.get("ok") is not True:
        return False
    status = response.get("status")
    return (
        isinstance(status, Mapping)
        and isinstance(status.get("error_code"), int)
        and not isinstance(status.get("error_code"), bool)
        and status.get("error_code") == 0
    )
