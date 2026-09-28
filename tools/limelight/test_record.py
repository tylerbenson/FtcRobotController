import contextlib
import io
import json
from pathlib import Path
import tempfile
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import record


class RecorderTests(unittest.TestCase):
    def test_usb_urls(self):
        self.assertEqual(record.base_url("limelight.local"), "http://limelight.local:5807")
        self.assertEqual(record.base_url("172.29.0.1"), "http://172.29.0.1:5807")
        self.assertEqual(record.base_url("http://localhost:1234"), "http://localhost:1234")
        with self.assertRaises(ValueError):
            record.base_url("http://limelight.local/results")

    def test_duplicate_age_and_restart(self):
        clock = record.FrameClock()
        self.assertEqual(clock.observe({"ts": 7}, 10), (False, 0))
        self.assertEqual(clock.observe({"ts": 7}, 510), (True, 500))
        self.assertEqual(clock.observe({"ts": 0}, 520), (False, 0))
        self.assertEqual(clock.observe({}, 530), (None, None))

    def test_capture_against_http_camera(self):
        responses = [
            {"Results": {"v": 1, "ts": 10, "pID": 0, "Fiducial": [
                {"fID": 34, "pts": [[1, 1], [2, 1], [2, 2], [1, 2]]}]}},
            {"Results": {"v": 1, "ts": 10, "pID": 0, "Fiducial": [
                {"fID": 34, "pts": [[1, 1], [2, 1], [2, 2], [1, 2]]}]}},
            b"not json", None,
            {"v": 1, "ts": 11, "Fiducial": [{"fID": 38, "pts": []}]},
        ]
        class Handler(BaseHTTPRequestHandler):
            index = 0
            def do_GET(self):
                if self.path == "/results":
                    value = responses[min(Handler.index, len(responses) - 1)]
                    Handler.index += 1
                else:
                    value = {"firmware": "fixture", "endpoint": self.path}
                self.send_response(503 if value is None else 200)
                self.end_headers()
                self.wfile.write(value if isinstance(value, bytes) else json.dumps(value).encode())
            def log_message(self, *args):
                pass
        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with tempfile.TemporaryDirectory() as temporary, contextlib.redirect_stdout(io.StringIO()):
                args = record.parser().parse_args([
                    "record", "--host", f"127.0.0.1:{server.server_port}",
                    "--scenario", "red front", "--alliance", "red", "--side", "audience",
                    "--duration", "0.3", "--hz", "100", "--output", temporary])
                directory = record.record(args)
                rows = list(record.rows(directory / "frames.jsonl"))
                self.assertGreaterEqual(len(rows), 5)
                self.assertEqual(rows[0]["response"], responses[0])
                self.assertFalse(rows[0]["duplicate"])
                self.assertTrue(rows[1]["duplicate"])
                self.assertGreater(rows[1]["observed_staleness_ms"], 0)
                self.assertIn("error", rows[2])
                self.assertIn("error", rows[3])
                self.assertEqual(rows[4]["tags_with_corners"], 0)
                metadata = json.loads((directory / "session.json").read_text())
                self.assertTrue(metadata["complete"])
                self.assertIsNone(metadata["calibration"])
                self.assertIn("/cal-default", metadata["snapshots"])
                summary = json.loads((directory / "summary.json").read_text())
                self.assertEqual(summary["errors"], 2)
                self.assertEqual(summary["observed_new_frames"], 2)
                self.assertGreater(summary["polls_with_missing_corners"], 0)
                self.assertTrue((directory / "frames.csv").exists())
        finally:
            server.shutdown()
            server.server_close()
            thread.join()

    def test_interrupted_session(self):
        with tempfile.TemporaryDirectory() as temporary, contextlib.redirect_stdout(io.StringIO()):
            path = Path(temporary) / "frames.jsonl"
            path.write_text('{"seq": 0}\n{"seq":')
            self.assertEqual(list(record.rows(path)), [{"seq": 0}])
            path.write_text('{"seq": 0}\ninvalid\n')
            with self.assertRaises(ValueError):
                list(record.rows(path))


if __name__ == "__main__":
    unittest.main()
