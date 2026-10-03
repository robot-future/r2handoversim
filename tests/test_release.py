import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from r2handoversim.cli import main
from r2handoversim.doctor import inspect_environment
from r2handoversim.demos import load_demo
from r2handoversim.geometry import box, pose
from r2handoversim.planning import configuration_clear, plan_trial
from r2handoversim.robot import HOME, tcp


class ReleaseTests(unittest.TestCase):
    def test_doctor_offline_does_not_import_isaac(self):
        # Missing simulator is optional for offline users, required for demo users.
        import importlib.util
        original = importlib.util.find_spec
        with patch('importlib.util.find_spec', side_effect=lambda name: None if name == 'isaacsim' else original(name)):
            self.assertEqual(inspect_environment()['status'], 'passed')
            self.assertEqual(inspect_environment(require_isaac=True)['status'], 'failed')

    def test_missing_video_encoder_is_reported(self):
        with patch('r2handoversim.video.require_encoder', side_effect=ValueError('ffmpeg unavailable')):
            report = inspect_environment(video=True)
        self.assertEqual(report['status'], 'failed')
        self.assertIn('ffmpeg', report['checks'][-1]['detail'])

    def test_bad_trial_type_has_readable_cli_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'bad.json'; path.write_text('[null]')
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr), self.assertRaises(SystemExit) as raised:
                main(['evaluate','--trials',str(path),'--output',str(Path(tmp)/'out')])
            self.assertEqual(raised.exception.code, 2)
            self.assertIn('JSON object', stderr.getvalue())
            self.assertNotIn('Traceback', stderr.getvalue())

    def test_calibrated_object_collision_uses_calibrated_pose(self):
        trial = load_demo('bottle')
        trial['T_object_gripper'] = pose().tolist()
        trial['T_tcp_asset_tool'] = pose([0,0,.5]).tolist()
        trial['object_boxes'] = [box([0,0,0],[.02]*3)]
        trial['hand_boxes_world'] = [box([8,8,8],[.01]*3)]
        center = (tcp(HOME)@np.array(trial['T_tcp_asset_tool']))[:3,3]
        trial['obstacle_boxes_world'] = [box(center,[.01]*3)]
        self.assertFalse(configuration_clear(trial,HOME))
        trial.pop('T_tcp_asset_tool')
        self.assertTrue(configuration_clear(trial,HOME))

    def test_calibrated_ik_solves_nominal_tcp_target(self):
        trial = load_demo('bottle')
        offset = pose([0,0,.04]); trial['T_tcp_asset_tool'] = offset.tolist()
        trial['target_T_world_gripper'] = (tcp(HOME)@offset).tolist()
        # Isolate the frame conversion from collision/path search.
        with patch('r2handoversim.planning.solve_pose', return_value=[] ) as solve:
            plan_trial(trial,iterations=1)
        np.testing.assert_allclose(solve.call_args.args[0],tcp(HOME),atol=1e-12)

    def test_standalone_mesh_generator(self):
        import trimesh
        from r2handoversim.asset_demos import generate
        from r2handoversim.evaluation import validate_trial
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); (root/'robot.usda').write_text('#usda 1.0')
            trimesh.creation.box([.04,.03,.18]).export(root/'sample.obj')
            config = root/'assets.json'
            config.write_text(json.dumps(dict(schema_version='handover.local_assets.v1',robot_usd='robot.usda',object_mesh_root='.')))
            trials = generate(config,root/'out')
            self.assertEqual(len(trials),1)
            validate_trial(trials[0])
            self.assertEqual(trials[0]['object_id'],'sample')
            self.assertEqual(trials[0]['split'],'S0')
            self.assertTrue(trials[0]['object_mesh_object']['faces'])
            self.assertEqual(len(trials[0]['executed_joints']),90)
            self.assertTrue((root/'out'/'trials.json').is_file())

    def test_export_audit_detects_changed_observations(self):
        from r2handoversim.evaluation import evaluate
        from r2handoversim.trajectory_export import export
        from r2handoversim.verification import verify_output
        trial = load_demo('bottle')
        trial['executed_joints'] = trial['executed_joints'][:2]
        trial['planned_joints'] = trial['planned_joints'][:2]
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            result = evaluate(trial, [False,False])
            result['artifacts'] = export(trial,[False,False],root)
            (root/'run.json').write_text(json.dumps(dict(status='succeeded',expected_trials=1,completed_trials=1)))
            (root/'results.json').write_text(json.dumps([result]))
            self.assertEqual(verify_output(root)['frames'],2)
            path = root/result['artifacts']['trajectory']
            with np.load(path,allow_pickle=False) as data:
                arrays = dict(data)
            arrays['robot_hand_contact'][1] = True
            np.savez_compressed(path,**arrays)
            with self.assertRaisesRegex(ValueError,'differs'):
                verify_output(root)
