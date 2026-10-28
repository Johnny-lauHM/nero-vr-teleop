from __future__ import annotations

import unittest

from nerox_openarmx_teleop.payload import (
    PAYLOAD_CONFIG_CAN_ID,
    PayloadConfigResult,
    configure_payload,
    pack_standard_can_frame,
    payload_config_data,
    unpack_standard_can_frame,
)


class FakeCanSocket:
    def __init__(self) -> None:
        self.sent: list[bytes] = []

    def __enter__(self) -> "FakeCanSocket":
        return self

    def __exit__(self, *_args: object) -> None:
        return None

    def recv(self, _size: int) -> bytes:
        raise BlockingIOError

    def send(self, frame: bytes) -> int:
        self.sent.append(frame)
        return len(frame)


class PayloadProtocolTests(unittest.TestCase):
    def test_1010g_half_load_uses_nero_v121_bytes(self) -> None:
        self.assertEqual(
            payload_config_data("half"),
            bytes.fromhex("00 00 00 AE 01 00 00 00"),
        )

    def test_payload_codes_are_not_piper_offset_values(self) -> None:
        self.assertEqual(payload_config_data("empty")[4], 0x00)
        self.assertEqual(payload_config_data("half")[4], 0x01)
        self.assertEqual(payload_config_data("full")[4], 0x02)

    def test_write_only_configuration_sends_once_and_marks_unverified(self) -> None:
        fake = FakeCanSocket()
        result = configure_payload(
            "can_arm_a",
            level="half",
            mass_kg=1.010,
            socket_factory=lambda _port: fake,
        )
        self.assertEqual(
            result,
            PayloadConfigResult(
                "can_arm_a", "half", 1.010, True, False, 1
            ),
        )
        self.assertEqual(len(fake.sent), 1)
        can_id, data = unpack_standard_can_frame(fake.sent[0])
        self.assertEqual(can_id, PAYLOAD_CONFIG_CAN_ID)
        self.assertEqual(data, payload_config_data("half"))

    def test_can_frame_round_trip(self) -> None:
        data = payload_config_data("half")
        frame = pack_standard_can_frame(PAYLOAD_CONFIG_CAN_ID, data)
        self.assertEqual(unpack_standard_can_frame(frame), (0x477, data))

    def test_invalid_level_is_rejected_before_socket_open(self) -> None:
        with self.assertRaisesRegex(ValueError, "empty, half, full"):
            configure_payload(
                "can_arm_a",
                level="1010g",
                socket_factory=lambda _port: self.fail("socket opened"),
            )


if __name__ == "__main__":
    unittest.main()
