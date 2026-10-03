import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from r2handoversim.demos import load_demo
from r2handoversim.geometry import box, pose
from r2handoversim.workflows import from_experiment, from_pipeline


class WorkflowTests(unittest.TestCase):
    def test_receiver_is_fixed_across_different_grasps(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            scene={'schema_version':'handover.scene.v1','units':'m',
                'object':{'id':'test','boxes':[box([0,0,0],[.03,.02,.07])],
                          'usage_regions':{'zone':[box([0,0,-.05],[.03,.02,.02])]}},
                'intent':{'human_region':'zone'},
                'receiving_hand':{'center':[-.08,0,0],'boxes':[box([-.08,0,0],[.01]*3)]}}
            (root/'scene.json').write_text(json.dumps(scene))
            for mode,t in [('FS',pose([0,0,.07])),('A1',pose([.03,0,0],[[0,0,1],[0,1,0],[-1,0,0]]))]:
                (root/f'{mode}.json').write_text(json.dumps({'schema_version':'handover.selection.v1','units':'m',
                    'status':'ok','object_id':'test','mode':mode,'selected':{'id':mode,'T_object_gripper':t.tolist()}}))
            manifest={'schema_version':'handover.experiment.v1','modes':['FS','A1'],'objects':[{'object_id':'test',
                       'scene':'scene.json','selections':{'FS':'FS.json','A1':'A1.json'}}]}
            (root/'experiment.json').write_text(json.dumps(manifest))
            def fake_plan(trial,**kwargs):
                trial['planning']={'status':'failed'};return trial
            with patch('r2handoversim.planning.plan_trial',side_effect=fake_plan):
                trials,state=from_experiment(root/'experiment.json',root/'out')
            self.assertEqual(trials[0]['hand_boxes_world'],trials[1]['hand_boxes_world'])
            np.testing.assert_allclose(trials[0]['delivery']['T_world_object'],trials[1]['delivery']['T_world_object'])
            self.assertFalse(np.allclose(trials[0]['target_T_world_gripper'],trials[1]['target_T_world_gripper']))
            self.assertEqual(state['planning_failures'],2)
            self.assertEqual(len(json.loads((root/'out/trials.json').read_text())),2)

    def test_incomplete_pipeline_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'pipeline.json';path.write_text('{"schema_version":"handover.pipeline.v1","status":"failed"}')
            with self.assertRaisesRegex(ValueError,'completed'): from_pipeline(path)

    def test_offline_duplicate_trials_are_rejected(self):
        from r2handoversim.cli import main
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'trials.json';trial=load_demo('hammer');path.write_text(json.dumps([trial,trial]))
            with self.assertRaises(SystemExit) as exc:
                main(['evaluate','--trials',str(path),'--output',str(Path(tmp)/'out')])
            self.assertEqual(exc.exception.code,2)
            self.assertFalse((Path(tmp)/'out/results.json').exists())

    def test_neural_asset_pipeline_preserves_delivery_and_defers_mesh_planning(self):
        scene={'schema_version':'handover.scene.v1','units':'m','object':{'id':'bottle',
            'boxes':[box([0,0,0],[.02]*3)],'usage_regions':{'all':[box([0,0,0],[.02]*3)]},
            'mesh':{'vertices':[[0,0,0],[.02,0,0],[0,.02,0]],'faces':[[0,1,2]]}},
            'intent':{'human_region':'all'},'gripper':{'max_opening_m':.085,'geometry_contract':{
                'robot':{'usd':'local.usd','translation':[0,0,0]}}},
            'receiving_hand':{'center':[.1,0,0],'boxes':[box([.1,0,0],[.02]*3)],
                'mesh':{'vertices':[[.1,0,0],[.1,.02,0],[.1,0,.02]],'faces':[[0,1,2]]}}}
        target=pose([-.4,-.3,1]).tolist()
        selection={'schema_version':'handover.selection.v1','object_id':'bottle','status':'ok','mode':'FS',
            'grasp_contract':{'width_policy':'asset_mesh_pad'},
            'selected':{'id':'g','T_object_gripper':np.eye(4).tolist(),'width_m':.03}}
        delivery={'schema_version':'handover.delivery.v1','units':'m','object_id':'bottle','grasp_id':'g',
            'T_world_object':target,'T_world_gripper':target}
        pipeline={'schema_version':'handover.pipeline.v1','status':'succeeded',
            'files':{'scene':'scene.json','selection':'selection.json','delivery':'delivery.json'}}
        with tempfile.TemporaryDirectory() as tmp, patch('r2handoversim.planning.plan_trial') as proxy:
            root=Path(tmp)
            for name,value in [('scene',scene),('selection',selection),('delivery',delivery),('pipeline',pipeline)]:
                (root/f'{name}.json').write_text(json.dumps(value))
            trial=from_pipeline(root/'pipeline.json',seed=27)
            scene['receiving_hand'].pop('mesh')
            (root/'scene.json').write_text(json.dumps(scene))
            with self.assertRaisesRegex(ValueError,'receiving_hand.mesh'):
                from_pipeline(root/'pipeline.json',seed=27)
        proxy.assert_not_called()
        self.assertEqual(trial['target_T_world_object'],target)
        self.assertEqual(trial['asset_robot'],scene['gripper']['geometry_contract']['robot'])
        self.assertTrue(trial['receiver_protocol']['replan_in_isaac'])
        self.assertEqual(trial['receiver_protocol']['hand_collision'],'mesh')
        self.assertEqual(trial['receiver_protocol']['planning_seed'],27)
        self.assertEqual(len(trial['pipeline_reference']['sha256']),64)
