"""Minimal NERO V1.2.1 firmware payload configuration.

This module is deliberately self-contained.  NERO's payload codes differ
from the legacy Piper enum carried by pyAgxArm, so the exact 0x477 bytes are
kept here and covered by tests.
"""

from __future__ import annotations

import argparse
import math
import select
import socket
import struct
import time
from dataclasses import dataclass
from typing import Any, Callable


PAYLOAD_CONFIG_CAN_ID = 0x477
SET_INSTRUCTION_ACK_CAN_ID = 0x476
SET_INSTRUCTION_PAYLOAD_INDEX = 0x77
PAYLOAD_LEVEL_CODES = {"empty": 0x00, "half": 0x01, "full": 0x02}

CAN_SFF_MASK = 0x7FF
CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_ERR_FLAG = 0x20000000
CAN_FRAME_STRUCT = struct.Struct("=IB3x8s")
SOL_CAN_RAW = 101
CAN_RAW_FILTER = 1


@dataclass(frozen=True)
class PayloadConfigResult:
    can_port: str
    level: str
    mass_kg: float
    transmitted: bool
    verified: bool
    attempts: int


def payload_config_data(level: str) -> bytes:
    """Return the exact NERO V1.2.1 bytes for CAN command 0x477."""

    try:
        code = PAYLOAD_LEVEL_CODES[level]
    except KeyError as exc:
        allowed = ", ".join(PAYLOAD_LEVEL_CODES)
        raise ValueError(f"payload level must be one of: {allowed}") from exc
    return bytes((0x00, 0x00, 0x00, 0xAE, code, 0x00, 0x00, 0x00))


def pack_standard_can_frame(can_id: int, data: bytes) -> bytes:
    if not 0 <= can_id <= CAN_SFF_MASK:
        raise ValueError("can_id must be an 11-bit standard CAN identifier")
    if len(data) > 8:
        raise ValueError("classic CAN data must not exceed 8 bytes")
    return CAN_FRAME_STRUCT.pack(can_id, len(data), data.ljust(8, b"\x00"))


def unpack_standard_can_frame(frame: bytes) -> tuple[int, bytes]:
    if len(frame) != CAN_FRAME_STRUCT.size:
        raise ValueError(f"classic CAN frame must be {CAN_FRAME_STRUCT.size} bytes")
    raw_can_id, data_length, data = CAN_FRAME_STRUCT.unpack(frame)
    if raw_can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG):
        raise ValueError("expected a standard CAN data frame")
    if data_length > 8:
        raise ValueError("invalid classic CAN data length")
    return raw_can_id & CAN_SFF_MASK, data[:data_length]


def is_payload_setting_ack(can_id: int, data: bytes) -> bool:
    return (
        can_id == SET_INSTRUCTION_ACK_CAN_ID
        and len(data) >= 1
        and data[0] == SET_INSTRUCTION_PAYLOAD_INDEX
    )


def _open_socket(can_port: str) -> socket.socket:
    can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    can_filter = struct.pack(
        "=II",
        SET_INSTRUCTION_ACK_CAN_ID,
        CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG,
    )
    can_socket.setsockopt(SOL_CAN_RAW, CAN_RAW_FILTER, can_filter)
    can_socket.bind((can_port,))
    can_socket.setblocking(False)
    return can_socket


def _drain(can_socket: Any) -> None:
    while True:
        try:
            can_socket.recv(CAN_FRAME_STRUCT.size)
        except BlockingIOError:
            return


def _wait_for_ack(can_socket: Any, deadline: float) -> bool:
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        readable, _, _ = select.select([can_socket], [], [], remaining)
        if not readable:
            return False
        try:
            can_id, data = unpack_standard_can_frame(
                can_socket.recv(CAN_FRAME_STRUCT.size)
            )
        except (BlockingIOError, ValueError):
            continue
        if is_payload_setting_ack(can_id, data):
            return True


def configure_payload(
    can_port: str,
    *,
    level: str = "half",
    mass_kg: float = 1.010,
    require_ack: bool = False,
    ack_timeout_s: float = 0.5,
    attempts: int = 3,
    socket_factory: Callable[[str], Any] = _open_socket,
) -> PayloadConfigResult:
    """Transmit one coarse NERO payload level and optionally require its ACK."""

    data = payload_config_data(level)
    mass = float(mass_kg)
    if not math.isfinite(mass) or mass < 0.0:
        raise ValueError("payload mass_kg must be non-negative and finite")
    if not math.isfinite(ack_timeout_s) or ack_timeout_s <= 0.0:
        raise ValueError("ack_timeout_s must be positive and finite")
    if attempts <= 0:
        raise ValueError("attempts must be positive")

    command = pack_standard_can_frame(PAYLOAD_CONFIG_CAN_ID, data)
    with socket_factory(can_port) as can_socket:
        _drain(can_socket)
        total_attempts = attempts if require_ack else 1
        for attempt in range(1, total_attempts + 1):
            can_socket.send(command)
            if not require_ack:
                return PayloadConfigResult(
                    can_port, level, mass, True, False, attempt
                )
            if _wait_for_ack(can_socket, time.monotonic() + ack_timeout_s):
                return PayloadConfigResult(
                    can_port, level, mass, True, True, attempt
                )
    return PayloadConfigResult(
        can_port, level, mass, True, False, total_attempts
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Configure NERO's coarse firmware payload compensation"
    )
    parser.add_argument("--can-port", action="append", required=True)
    parser.add_argument(
        "--payload-level", choices=tuple(PAYLOAD_LEVEL_CODES), default="half"
    )
    parser.add_argument("--payload-mass-kg", type=float, default=1.010)
    parser.add_argument("--require-ack", action="store_true")
    parser.add_argument("--ack-timeout-s", type=float, default=0.5)
    parser.add_argument("--attempts", type=int, default=3)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    for can_port in args.can_port:
        try:
            result = configure_payload(
                can_port,
                level=args.payload_level,
                mass_kg=args.payload_mass_kg,
                require_ack=args.require_ack,
                ack_timeout_s=args.ack_timeout_s,
                attempts=args.attempts,
            )
        except (OSError, ValueError) as exc:
            print(f"[nerox-payload] {can_port}: FAILED: {exc}")
            return 1
        if args.require_ack and not result.verified:
            print(
                f"[nerox-payload] {can_port}: no 0x476/0x77 ACK after "
                f"{result.attempts} attempts"
            )
            return 1
        verification = "ACK verified" if result.verified else (
            "UNVERIFIED: NERO 1.121 has no payload ACK/readback"
        )
        print(
            f"[nerox-payload] {can_port}: transmitted {result.level} for "
            f"{result.mass_kg:.3f}kg; {verification}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
