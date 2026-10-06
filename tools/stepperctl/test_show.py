"""Show compiler tests (no hardware)."""

import math
import struct
import unittest

from stepperctl import show


class ShowTests(unittest.TestCase):
    def test_golden(self):
        # firmware/test/test_host.c parses these same bytes.
        tiny = show.compile_show({"name": "x", "duration": 2000, "loop": True,
                                  "axes": {"3": [[0, 1.5], [1000, -2.25, 10]]},
                                  "leds": {"12": [[0, "#102030"]]}})
        self.assertEqual(tiny.hex(), "53484f570100280050000000d69d43c578000000000000000000000000000000d00700000102000001020200000000000000c03f0000c07fe8030000000010c000002041020c01000000000010203000")

    def test_header_and_crc(self):
        b = show.compile_show({"name": "t", "axes": {"1": [[0, 0], [500, 10]]}})
        magic, ver, hsize, length, crc = struct.unpack_from("<IHHII", b)
        self.assertEqual((magic, ver, hsize, length), (show.MAGIC, 1, 40, len(b)))
        self.assertEqual(crc, show.crc32(b))
        _, _, n_keys = struct.unpack_from("<BBH", b, 40)
        t, pos, vel = struct.unpack_from("<Iff", b, 44)
        self.assertEqual((n_keys, t, pos), (2, 0, 0.0))
        self.assertTrue(math.isnan(vel))  # automatic speed

    def test_examples_compile(self):
        import os
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        for name in ("hello.json", "wave.json"):
            self.assertLess(len(show.load(os.path.join(here, "shows", name))), show.MAX_SIZE)

    def test_errors(self):
        bad = [
            {"axes": {"11": [[0, 0]]}},                                   # axis out of range
            {"axes": {"1": [[0, 0], [0, 1]]}},                            # times must increase
            {"loop": True, "duration": 1000, "axes": {"1": [[0, 0], [1000, 1]]}},  # no seam
            {"leds": {"25": [[0, "#000000"]]}},                           # pixel out of range
            {"leds": {"1": [[0, "#12345"]]}},                             # bad colour
            {"name": "x" * 17},
        ]
        for spec in bad:
            with self.assertRaises(show.ShowError, msg=spec):
                show.compile_show(spec)


if __name__ == "__main__":
    unittest.main()
