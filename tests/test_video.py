from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch
from r2handoversim.video import encode_frames, complete_png
from r2handoversim.runner import replay
from r2handoversim.demos import load_demo


class VideoTests(unittest.TestCase):
    def test_capture_waits_for_png_end_marker(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'frame.png'
            self.assertFalse(complete_png(path))
            path.write_bytes(b'partial')
            self.assertFalse(complete_png(path))
            path.write_bytes(b'png-data\x00\x00\x00\x00IEND\xaeB`\x82')
            self.assertTrue(complete_png(path))

    def test_missing_encoder_is_reported_before_kit_starts(self):
        with tempfile.TemporaryDirectory() as tmp, patch('r2handoversim.video.shutil.which', return_value=None):
            with self.assertRaisesRegex(ValueError, 'ffmpeg'):
                replay([load_demo('hammer')], tmp, video=True)
            self.assertFalse((Path(tmp)/'run.json').exists())

    def test_missing_frame_does_not_replace_existing_video(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);video=root/'demo.mp4';video.write_bytes(b'previous')
            with self.assertRaisesRegex(RuntimeError, 'Missing recorded'):
                encode_frames(root, video, 2, 1/60)
            self.assertEqual(video.read_bytes(), b'previous')

    def test_encoder_failure_preserves_previous_video_and_removes_partial(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);video=root/'demo.mp4';video.write_bytes(b'previous')
            (root/'000000.png').write_bytes(b'frame')
            def fail(*args, **kwargs):
                (root/'demo.partial.mp4').write_bytes(b'partial')
                raise subprocess.CalledProcessError(1, 'ffmpeg', stderr=b'codec failure')
            with patch('r2handoversim.video.require_encoder', return_value='ffmpeg'), patch('r2handoversim.video.subprocess.run', side_effect=fail):
                with self.assertRaisesRegex(RuntimeError, 'codec failure'):
                    encode_frames(root, video, 1, 1/60)
            self.assertEqual(video.read_bytes(), b'previous')
            self.assertFalse((root/'demo.partial.mp4').exists())
