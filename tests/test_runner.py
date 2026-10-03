import json
from pathlib import Path
import tempfile
import unittest
from r2handoversim.runner import check_completion, replay
from r2handoversim.demos import load_demo
from r2handoversim.evaluation import evaluate, validate_trial
from r2handoversim.report import write_report


class RunnerTests(unittest.TestCase):
    def test_exit_zero_does_not_hide_incomplete_run(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)
            (p/'run.json').write_text(json.dumps({'run_id':'current','status':'failed','completed_trials':0,'error':'capture failed'}))
            with self.assertRaisesRegex(RuntimeError,'capture failed'):
                check_completion(p,'current',1,0)

    def test_stale_results_cannot_validate_new_run(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)
            (p/'run.json').write_text(json.dumps({'run_id':'old','status':'succeeded','completed_trials':1}))
            (p/'results.json').write_text('[{}]')
            with self.assertRaises(RuntimeError):
                check_completion(p,'new',1,0)
            self.assertEqual(check_completion(p,'old',1,0),[{}])

    def test_missing_results_cannot_pass(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(RuntimeError):
                check_completion(tmp,'new',1,0)

    def test_duplicate_ids_fail_before_simulator_start(self):
        with tempfile.TemporaryDirectory() as tmp:
            trial=load_demo('hammer')
            with self.assertRaisesRegex(ValueError,'Duplicate'):
                replay([trial,trial],tmp)

    def test_invalid_mesh_is_rejected(self):
        trial=load_demo('hammer')
        trial['hand_mesh_world']={'vertices':[[0,0,0],[1,0,0],[0,1,0]],'faces':[[0,1,3]]}
        with self.assertRaisesRegex(ValueError,'mesh'):
            validate_trial(trial)

    def test_contact_strings_are_not_truthy_physics_readings(self):
        trial=load_demo('hammer')
        with self.assertRaisesRegex(ValueError,'boolean'):
            evaluate(trial,['false']*len(trial['executed_joints']))

    def test_report_does_not_link_stale_artifacts(self):
        result={"trial_id":"case","split":"S0","success":True,"first_failure":None,
                "metrics":{"stability":True,"plan":True,"reach":True,"affordance":None,"safe":True}}
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)
            (p/'case.png').touch()
            write_report([result],p)
            self.assertNotIn('href="case.png"',(p/'report.html').read_text())
            result['artifacts']={'screenshot':'case.png'}
            write_report([result],p)
            self.assertIn('href="case.png"',(p/'report.html').read_text())


if __name__ == '__main__':
    unittest.main()
