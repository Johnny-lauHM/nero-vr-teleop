from __future__ import annotations

import math
from typing import Any

from nerox_openarmx_teleop.tacclaw_remote import (
    RemoteCalibration,
    TacClawEndpoint,
    TacClawRemoteClient,
    TacClawRemoteWorker,
    read_token,
)


def trigger_to_position(trigger: float) -> int:
    """Map Quest trigger 0..1 to TacClaw 1000(open)..0(closed)."""

    value = float(trigger)
    if not math.isfinite(value):
        raise ValueError("TacClaw trigger must be finite")
    value = min(1.0, max(0.0, value))
    return int(round((1.0 - value) * 1000.0))


class TacClawPair:
    """Own two authenticated board-side TacClaw workers."""

    def __init__(self, args: Any) -> None:
        token = read_token(args.tacclaw_token_path)
        workers = {}
        for side, host, closed_raw, open_raw in (
            (
                "left",
                args.tacclaw_left_host,
                args.tacclaw_left_closed_raw,
                args.tacclaw_left_open_raw,
            ),
            (
                "right",
                args.tacclaw_right_host,
                args.tacclaw_right_closed_raw,
                args.tacclaw_right_open_raw,
            ),
        ):
            endpoint = TacClawEndpoint(
                side=side,
                host=host,
                port=args.tacclaw_port,
                calibration=RemoteCalibration(
                    closed_raw=closed_raw,
                    open_raw=open_raw,
                ),
                torque=args.tacclaw_torque_limit,
                speed=args.tacclaw_speed,
                watchdog_s=args.tacclaw_watchdog_s,
            )
            client = TacClawRemoteClient(
                endpoint,
                token=token,
                connect_timeout_s=args.tacclaw_connect_timeout_s,
                request_timeout_s=args.tacclaw_request_timeout_s,
            )
            workers[side] = TacClawRemoteWorker(
                client,
                name=side,
                command_rate_hz=args.tacclaw_command_rate_hz,
                position_deadband=args.tacclaw_position_deadband,
                initial_position=args.tacclaw_initial_position,
                tracking_timeout_s=args.tacclaw_tracking_timeout_s,
                tracking_tolerance=args.tacclaw_tracking_tolerance,
            )
        self.workers = workers
        started = []
        try:
            for worker in self.workers.values():
                worker.start(clearance_confirmed=True)
                started.append(worker)
        except Exception:
            for worker in reversed(started):
                worker.close(timeout_s=2.0)
            raise

    @property
    def ready(self) -> bool:
        return all(worker.ready for worker in self.workers.values())

    def fault_message(self) -> str | None:
        failures = [
            f"{side}={worker.state.value}:{worker.error}"
            for side, worker in self.workers.items()
            if worker.state.value in {"fault", "closed"}
        ]
        return "; ".join(failures) if failures else None

    def snapshot(self) -> dict[str, dict[str, Any]]:
        result: dict[str, dict[str, Any]] = {}
        for side, worker in self.workers.items():
            with worker.condition:
                result[side] = {
                    "state": worker._state.value,
                    "ready": worker._state.value == "ready",
                    "error": worker._error,
                    "measured_position": worker.measured_position,
                    "commanded_position": worker.last_commanded_position,
                    "command_sequence": getattr(worker, "command_sequence", 0),
                    "command_send_wall_ns": getattr(worker, "command_send_wall_ns", None),
                    "command_send_monotonic_ns": getattr(
                        worker, "command_send_monotonic_ns", None
                    ),
                    "command_ack_wall_ns": getattr(worker, "command_ack_wall_ns", None),
                    "command_ack_monotonic_ns": getattr(
                        worker, "command_ack_monotonic_ns", None
                    ),
                }
        return result

    def command(self, side: str, trigger: float) -> int:
        position = trigger_to_position(trigger)
        return self.workers[side].command_position(
            position, clearance_confirmed=True
        )

    def cancel_pending(self) -> None:
        for worker in self.workers.values():
            worker.cancel_pending()

    def close(self) -> None:
        errors = []
        for side, worker in self.workers.items():
            try:
                worker.close(timeout_s=2.0)
            except Exception as exc:  # close both sides even if one SDK blocks
                errors.append(f"{side}: {exc}")
        if errors:
            raise RuntimeError("TacClaw shutdown failed: " + "; ".join(errors))
