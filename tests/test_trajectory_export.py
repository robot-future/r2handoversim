import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from r2handoversim.demos import load_demo
from r2handoversim.evaluation import trial_tcp
from r2handoversim.geometry import inverse
from r2handoversim.trajectory_export import export


class TrajectoryExportTests(unittest.TestCase):
    def test_frame_timing_contacts_and_object_pose_match_the_resolved_grasp(self):
        trial=load_demo('hammer');n=len(trial['executed_joints']);contacts=[False]*n;contacts[2]=True
        with tempfile.TemporaryDirectory() as tmp:
            files=export(trial,contacts,tmp)
            resolved=json.loads((Path(tmp)/files['trial']).read_text())
            self.assertEqual(resolved,trial)
            with np.load(Path(tmp)/files['trajectory'],allow_pickle=False) as data:
                self.assertAlmostEqual(data['time_s'][-1],(n-1)*trial['dt_s'])
                self.assertEqual(np.flatnonzero(data['robot_hand_contact']).tolist(),[2])
                expected=trial_tcp(trial,trial['executed_joints'][-1])@inverse(trial['T_object_gripper'])
                np.testing.assert_allclose(data['T_world_object'][-1],expected)

    def test_missing_observations_or_disagreeing_robot_motion_are_rejected(self):
        trial=load_demo('hammer');n=len(trial['executed_joints'])
        observed=np.array([trial_tcp(trial,q) for q in trial['executed_joints']]);observed[0,0,3]+=.01
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(ValueError,'boolean contact'):
                export(trial,[False],tmp)
            with self.assertRaisesRegex(ValueError,'differs'):
                export(trial,[False]*n,tmp,observed)
            self.assertEqual(list(Path(tmp).iterdir()),[])

    def test_reference_outcome_stays_separate_from_measured_csv_result(self):
        import csv
        from r2handoversim.evaluation import evaluate
        from r2handoversim.results import save_results
        trial=load_demo('hammer')
        result=evaluate(trial,[False]*len(trial['executed_joints']))
        result['replay_reference']={'assigned_outcome':'safe'}
        result['grasp_contact']={'bilateral_distance_m':[.00001,.00002]}
        with tempfile.TemporaryDirectory() as tmp:
            save_results([result],tmp)
            with (Path(tmp)/'results.csv').open() as stream: row=next(csv.DictReader(stream))
            self.assertEqual(row['reference_outcome'],'safe')
            self.assertEqual(row['success'],'True')
            self.assertEqual(row['first_failure'],'')
            self.assertEqual(float(row['right_pad_distance_m']),.00001)
            self.assertEqual(float(row['left_pad_distance_m']),.00002)
