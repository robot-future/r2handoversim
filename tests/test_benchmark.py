import copy
import unittest
import numpy as np
from r2handoversim.demos import load_demo
from r2handoversim.evaluation import evaluate, summarize
from r2handoversim.geometry import pose
from r2handoversim.robot import GOAL, frames, tcp


class BenchmarkTests(unittest.TestCase):
    def test_expected_failure_modes(self):
        expected = {"intent_aware": None, "region_agnostic": "affordance",
                    "execution_deviation": "safe", "missed_delivery": "reach"}
        for variant, failure in expected.items():
            result = evaluate(load_demo("screwdriver",variant))
            self.assertEqual(result["first_failure"],failure,variant)

    def test_s0_excludes_affordance(self):
        trial = load_demo("bottle")
        # Deliberately put usage geometry everywhere; S0 must still omit the test.
        trial["usage_boxes"][0]["half_extents"] = [5,5,5]
        result = evaluate(trial)
        self.assertIsNone(result["metrics"]["affordance"])
        self.assertIsNone(summarize([result])["splits"]["S0"]["failure_rates"]["affordance"])

    def test_first_failure_precedence(self):
        trial = load_demo("hammer","execution_deviation")
        trial["max_opening_m"] = .001
        self.assertEqual(evaluate(trial)["first_failure"],"stability")

    def test_plan_must_reach_target(self):
        trial = load_demo("hammer")
        trial["target_T_world_gripper"] = pose([99,99,99]).tolist()
        self.assertEqual(evaluate(trial)["first_failure"],"plan")

    def test_physics_trace_coverage(self):
        trial = load_demo("hammer")
        with self.assertRaises(ValueError):
            evaluate(trial, [False])

    def test_failure_rates_partition_trials(self):
        rows = [{"split":"S1","success":True,"first_failure":None},
                {"split":"S1","success":False,"first_failure":"safe"},
                {"split":"S1","success":False,"first_failure":"reach"}]
        summary=summarize(rows)["splits"]["S1"]
        self.assertAlmostEqual(summary["success_rate"]+sum(summary["failure_rates"].values()),1.)

    def test_kinematics_rigid_and_continuous(self):
        for t in frames(GOAL):
            np.testing.assert_allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-10)
        self.assertLess(np.linalg.norm(tcp(GOAL)[:3,3]-tcp(GOAL+1e-5)[:3,3]),1e-3)


if __name__ == "__main__":
    unittest.main()
