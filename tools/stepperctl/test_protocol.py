"""Protocol tests (no hardware): python3 -m unittest discover -s tools/stepperctl -p 'test_*.py' -t tools"""

import random
import struct
import unittest

from stepperctl import protocol as P


class FrameTests(unittest.TestCase):
    def test_crc_check_value(self):
        self.assertEqual(P.crc16(b"123456789"), 0x29B1)

    def test_cobs_vectors(self):
        vectors = [
            (b"\x00", b"\x01\x01"),
            (b"\x00\x00", b"\x01\x01\x01"),
            (b"\x11\x22\x00\x33", b"\x03\x11\x22\x02\x33"),
            (b"\x11\x22\x33\x44", b"\x05\x11\x22\x33\x44"),
            (b"\x11\x00\x00\x00", b"\x02\x11\x01\x01\x01"),
            (bytes(range(1, 255)), b"\xff" + bytes(range(1, 255))),
            (bytes(range(1, 256)), b"\xff" + bytes(range(1, 255)) + b"\x02\xff"),
        ]
        for raw, enc in vectors:
            self.assertEqual(P.cobs_encode(raw), enc)
            self.assertEqual(P.cobs_decode(enc), raw)

    def test_golden_frame(self):
        # Same bytes as firmware/test/test_host.c test_frame().
        frame = P.build_frame(0x10, 0x1234, b"\x00\x01\x02\xff")
        self.assertEqual(frame.hex(" "), "00 04 10 34 12 06 01 02 ff e3 e0 00")

    def test_round_trip_and_corruption(self):
        rng = random.Random(7)
        for k in range(500):
            payload = bytes(0 if rng.random() < 0.25 else rng.randrange(256) for _ in range(rng.randrange(241)))
            frame = P.build_frame(k & 0xFF, k * 31 & 0xFFFF, payload)
            self.assertNotIn(0, frame[1:-1])
            self.assertEqual(P.parse_frame(frame[1:-1]), (k & 0xFF, k * 31 & 0xFFFF, payload))
            bad = bytearray(frame[1:-1])
            bad[rng.randrange(len(bad))] ^= rng.randrange(1, 256)
            try:
                self.assertNotEqual(P.parse_frame(bytes(bad)), (k & 0xFF, k * 31 & 0xFFFF, payload))
            except ValueError:
                pass


class MessageTests(unittest.TestCase):
    def test_request_sizes(self):
        # Must match the payload layouts in firmware/src/protocol.h.
        self.assertEqual(len(P.req_axes_pos(1, 1.0)), 10)
        self.assertEqual(len(P.req_jog(1, 1.0, 300)), 8)
        self.assertEqual(len(P.req_pvt_point(1, 1.0, 2.0, 20)), 16)
        self.assertEqual(len(P.req_group_line(0, [1, 2, 3])), 6 + 24)
        self.assertEqual(len(P.req_group_arc(0, 1, 2, 3.14)), 1 + 4 + 4 + 8 + 8 + 8)
        self.assertEqual(len(P.req_telemetry(100, 3, 1, 0, True)), 7)
        self.assertEqual(len(P.req_drive(1, 0.4, 0.6, 0.25, 300, 1600, True)), 2 + 5 * 4 + 1)

    def test_cam_info_layout(self):
        n = 10
        body = bytes([0b0101]) + b"".join(struct.pack("<bB", -1 if i % 2 else 0, 10) for i in range(n))
        body += b"".join(struct.pack("<qfffB", 3 << 30, 2.5, 150.0, 150.0, 2) for _ in range(2))
        info = P.parse_cam_info(body, n)
        self.assertEqual(info["loaded"], [0, 2])
        self.assertEqual(info["followers"][0], {"table": 0, "leader": 10})
        self.assertIsNone(info["followers"][1])
        self.assertEqual(info["leaders"][1]["pos"], 3.0)
        self.assertEqual(info["leaders"][0]["mode"], "vel")

    def test_units(self):
        self.assertEqual(P.to_units(1.0), 1 << 30)
        self.assertEqual(P.to_units(-12.5), -(25 << 29))

    def test_status_layout(self):
        n = 10
        body = struct.pack("<IBIIIHH", 5, 1, 7, 0, 0, 4700, 2200)
        for i in range(n):
            body += struct.pack("<qfBBBBb", (i + 1) << 30, 1.5, 1, 40, 3, 0, -1)
        for k in range(4):
            body += struct.pack("<BHBfI", 3, 0b11, 2, 800.0, 9)
        self.assertLessEqual(len(body), 240)  # FRAME_MAX_PAYLOAD
        st = P.parse_status(body, n)
        self.assertEqual(st.axes[2].pos, 3.0)
        self.assertTrue(st.axes[0].holding and st.axes[0].settled)
        self.assertEqual(st.groups[0].members, [1, 2])


if __name__ == "__main__":
    unittest.main()
