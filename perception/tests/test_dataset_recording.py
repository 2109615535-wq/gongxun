"""The completed video and its sampled frames survive an interrupted receipt."""

import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np

from rescue_vision.dashboard import make_server
from rescue_vision.dashboard_state import DashboardState
from rescue_vision.dataset_receive import DatasetReceiver, RECORDING_ENDPOINT
from rescue_vision.dataset_recording import DatasetRecorder
from rescue_vision.dual import load_dual_config


class DatasetRecordingTest(unittest.TestCase):
    def test_recording_is_received_once_and_samples_three_per_second(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            recorder = DatasetRecorder(root / "board")
            cfg = load_dual_config(Path(__file__).parents[1] / "config" / "dual_camera.json")
            state = DashboardState(cfg)
            state.camera_info["upper"] = {"reported_fps": 12}
            shape = (48, 64, 3)
            state.publish_raw("upper", np.zeros(shape, dtype=np.uint8), time.monotonic(), 1)
            server = make_server(state, "127.0.0.1", 0, recorder)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                url = f"http://127.0.0.1:{server.server_port}"
                receiver = DatasetReceiver(root / "dataset")
                (receiver.videos / "056.mp4").write_bytes(b"existing video")
                with receiver.request(url, RECORDING_ENDPOINT, {"recording": True}) as response:
                    self.assertTrue(json.load(response)["recording"])
                for frame in range(24):
                    recorder.offer(np.full(shape, frame * 8, dtype=np.uint8))
                self.assertEqual(receiver.pending(url), [])
                with receiver.request(url, RECORDING_ENDPOINT, {"recording": False}):
                    pass
                recorder.close()
                recording_id, = receiver.pending(url)

                original_request = receiver.request

                def interrupted_ack(base_url, endpoint, data=None):
                    if endpoint.endswith("/saved"):
                        raise OSError("interrupted")
                    return original_request(base_url, endpoint, data)

                with patch.object(receiver, "request", side_effect=interrupted_ack):
                    with self.assertRaisesRegex(OSError, "interrupted"):
                        receiver.receive(url, recording_id)
                self.assertEqual(receiver.pending(url), [recording_id])
                receiver.receive(url, recording_id)

                self.assertEqual(receiver.pending(url), [])
                self.assertEqual(sorted(path.name for path in receiver.videos.glob("*.mp4")),
                                 ["056.mp4", "057.mp4"])
                self.assertEqual(sorted(path.name for path in receiver.images.iterdir()),
                                 [f"057_{index:06d}.jpg" for index in range(1, 7)])
                self.assertEqual(recorder.snapshot()["last_saved"]["video"], "057.mp4")
                self.assertEqual(recorder.snapshot()["last_saved"]["images"], 6)
            finally:
                recorder.close()
                server.stop.set()
                server.shutdown()
                server.server_close()
                thread.join(timeout=2)
