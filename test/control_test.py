import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch
import threading


spec = importlib.util.spec_from_file_location(
    "control", Path(__file__).resolve().parents[1] / "py" / "control.py"
)
control = importlib.util.module_from_spec(spec)
spec.loader.exec_module(control)


class ControlTest(unittest.TestCase):
    def test_debug_response(self):
        self.assertEqual(
            control.format_response(b'{"type":"debug","event":"can_status"}\n'),
            '[debug] {"type":"debug","event":"can_status"}',
        )
        self.assertEqual(
            control.format_response(b'{"id":1,"ok":true,"debug":{"event":"can_tx_queued"}}\n'),
            '< {"id": 1, "ok": true}\n[debug] {"event": "can_tx_queued"}',
        )

    def test_other_serial_output(self):
        for line in (b'{"type":"ready"}', b'booting', b'[1,2]', b'bad\xff'):
            self.assertTrue(control.format_response(line).startswith("< "))

    def test_partial_serial_line(self):
        stopped = threading.Event()

        class Port:
            chunks = iter((b'{"type":"de', b'bug","event":"can_status"}\n'))

            def read_until(self, *args, **kwargs):
                try:
                    return next(self.chunks)
                except StopIteration:
                    stopped.set()
                    return b""

        with patch("builtins.print") as output:
            control.read_responses(Port(), stopped)
        output.assert_called_once_with(
            '\n[debug] {"type":"debug","event":"can_status"}', flush=True
        )

    def test_shortcuts(self):
        for text in ("throttle: 50", "throttle:50", " throttle : 50 "):
            self.assertEqual(
                control.parse_command(text), {"target": "vesc", "erpm": 50}
            )
        self.assertEqual(
            control.parse_command("steering: 50"),
            {"target": "steering", "position_deg": 50},
        )
        self.assertEqual(control.parse_command("throttle: -50")["erpm"], -50)
        self.assertEqual(control.parse_command("steering: -2.5")["position_deg"], -2.5)

    def test_numeric_fields(self):
        for text in ("vesc erpm: 1000", "vesc erpm=1000", " vesc erpm = 1000 "):
            self.assertEqual(
                control.parse_command(text), {"target": "vesc", "erpm": 1000}
            )
        self.assertIsInstance(control.parse_command("vesc erpm=1000")["erpm"], int)
        self.assertEqual(control.parse_command("vesc erpm=-2000")["erpm"], -2000)

    def test_steering(self):
        self.assertEqual(
            control.parse_command("steering position_deg=50"),
            {"target": "steering", "position_deg": 50},
        )
        self.assertEqual(
            control.parse_command("steering position_deg: -2.5")["position_deg"], -2.5
        )

    def test_generic_fields(self):
        self.assertEqual(
            control.parse_command('FutureNode enabled=true value=1e2 label="Keep: Case" empty=null'),
            {"target": "FutureNode", "enabled": True, "value": 100.0,
             "label": "Keep: Case", "empty": None},
        )
        self.assertEqual(control.parse_command("other mode=Manual")["mode"], "Manual")

    def test_invalid_commands(self):
        for text in (
            "", "throttle", "stop", "steering 50", "vesc erpm=",
            "throttle:", "steering: 50 extra", "unknown: 50",
            "vesc erpm=0 extra", "vesc erpm==0", 'other label="unfinished',
            "vesc id=5", "vesc target=other", "vesc erpm=0 erpm=1",
            "other value=NaN", "other value=Infinity", "other value=1e999",
        ):
            with self.subTest(text=text), self.assertRaises(ValueError):
                control.parse_command(text)


if __name__ == "__main__":
    unittest.main()
