import tempfile
from pathlib import Path
import unittest
import numpy as np
from r2handoversim.demos import load_demo
from r2handoversim.evaluation import validate_trial, trial_tcp
from r2handoversim.robot import HOME, tcp
from r2handoversim.local_assets import configuration


class LocalAssetTests(unittest.TestCase):
    def test_tool_offset_follows_wrist_not_world(self):
        trial = load_demo('hammer')
        offset = np.eye(4); offset[2,3] = .035
        trial['T_tcp_asset_tool'] = offset.tolist()
        np.testing.assert_allclose(trial_tcp(trial, HOME), tcp(HOME) @ offset)
        validate_trial(trial)

    def test_invalid_mesh_is_rejected_before_launch(self):
        trial = load_demo('hammer')
        trial['object_mesh_object'] = dict(vertices=[[0,0,0],[1,0,0],[0,1,0]], faces=[[0,1,9]], colors=[[1,1,1]]*3)
        with self.assertRaisesRegex(ValueError, 'object triangle'):
            validate_trial(trial)

    def test_missing_local_asset_is_explicit_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'config.json'
            path.write_text('{"schema_version":"handover.local_assets.v1","robot_usd":"missing.usd","object_mesh_root":"."}')
            with self.assertRaisesRegex(ValueError, 'Missing local asset'):
                configuration(path)
