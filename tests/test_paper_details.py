import copy
import unittest
import numpy as np
from r2handoversim.aggregation import table_rows
from r2handoversim.demos import load_demo
from r2handoversim.evaluation import evaluate
from r2handoversim.geometry import box, pose
from r2handoversim.planning import configuration_clear, plan_trial, rrt_connect, solve_pose
from r2handoversim.robot import GOAL, HOME, tcp
from r2handoversim.runner import replay


class PaperDetailsTests(unittest.TestCase):
    def test_full_pose_ik(self):
        goal=tcp(GOAL)
        solved=solve_pose(goal,attempts=2)
        self.assertTrue(solved)
        np.testing.assert_allclose(tcp(solved[0]),goal,atol=.001)

    def test_rrt_routes_around_obstacle(self):
        clear=lambda q: not (-.3 <= q[0] <= .3 and -.8 <= q[1] <= .8)
        path=rrt_connect([-1.,0.],[1.,0.],clear,np.random.default_rng(7),iterations=600)
        self.assertIsNotNone(path)
        self.assertTrue(all(clear(q) for q in path))
        self.assertGreater(max(abs(q[1]) for q in path),.8)
        np.testing.assert_allclose(path[0],[-1,0]); np.testing.assert_allclose(path[-1],[1,0])

    def test_obstacle_checks_include_attached_object(self):
        trial=load_demo('hammer')
        from r2handoversim.geometry import inverse, points
        center=points(tcp(HOME)@inverse(trial['T_object_gripper']),trial['object_boxes'][0]['center'])
        trial['obstacle_boxes_world']=[box(center,[.005]*3)]
        self.assertFalse(configuration_clear(trial,HOME))

    def test_unreachable_target_records_real_failure(self):
        trial=load_demo('hammer'); trial['target_T_world_gripper']=pose([9,9,9]).tolist()
        planned=plan_trial(trial,iterations=1)
        self.assertEqual(planned['planning']['status'],'failed')
        self.assertEqual(planned['planning']['reason'],'ik_failure')
        self.assertEqual(evaluate(planned)['first_failure'],'plan')

    def test_table_weights_objects_then_splits_and_missing_times(self):
        def row(obj,split,success):
            return {'object_id':obj,'split':split,'variant':'method','success':success,
                    'first_failure':None if success else 'affordance'}
        results=[row('a','S1',False)]*9+[row('b','S1',True),row('c','S0',True)]
        table={r['split']:r for r in table_rows(results)}
        self.assertEqual(table['S1']['SR'],50)
        self.assertEqual(table['Avg']['SR'],75)
        self.assertIsNone(table['S0']['Fafford'])
        self.assertEqual(table['Avg']['Fafford'],25)
        self.assertIsNone(table['Avg']['Tplan'])
        self.assertAlmostEqual(sum(table['Avg'][k] for k in ('Fplan','Freach','Fsafe','Fstab','Fafford')),100-table['Avg']['SR'])

    def test_mesh_mode_requires_mesh(self):
        with self.assertRaisesRegex(ValueError,'hand_mesh_world'):
            replay([load_demo('hammer')],'/tmp/unused-r2handover-output',hand_collision='mesh')

    def test_imported_points_and_explicit_split_survive_conversion(self):
        from r2handoversim.demos import from_selection
        cloud=[[0,0,0],[.01,0,0],[0,.01,0],[0,0,.01]]
        scene={'schema_version':'handover.scene.v1','object':{'id':'actual','boxes':[box([0,0,0],[.01]*3)],
            'surface_points':cloud,'usage_regions':{'zone':[box([0,0,0],[.005]*3)]}},
            'intent':{'human_region':'zone'},'receiving_hand':{'center':[-.1,0,0],'boxes':[box([-.1,0,0],[.01]*3)]},
            'evaluation_split':'S0','source_data':{'asset_sha256':'test'},'annotation_status':'generated'}
        selection={'schema_version':'handover.selection.v1','status':'ok','object_id':'actual',
                   'selected':{'id':'g','T_object_gripper':pose().tolist()}}
        trial=from_selection(scene,selection)
        self.assertEqual(trial['object_points_object'],cloud)
        self.assertEqual(trial['split'],'S0')
        self.assertEqual(trial['source_data']['asset_sha256'],'test')

    def test_invalid_imported_cloud_is_rejected(self):
        from r2handoversim.evaluation import validate_trial
        trial=load_demo('hammer');trial['object_points_object']=[[float('nan'),0,0]]*4
        with self.assertRaisesRegex(ValueError,'point cloud'): validate_trial(trial)
