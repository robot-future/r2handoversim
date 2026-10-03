from copy import deepcopy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from r2handoversim.demos import load_demo, from_selection
from r2handoversim.geometry import box, points, pose
from r2handoversim.receivers import generate
from r2handoversim.mesh_metrics import hull_reach
from r2handoversim.physx_planning import plan, verified_plan


class ReceiverTests(unittest.TestCase):
    def test_pose_bank_shared_across_modes_seeded_and_object_specific(self):
        t=load_demo('bottle');t['asset_robot']={'usd':'local.usd','translation':[0,0,0]}
        t['object_mesh_object']={'vertices':[[-.02,-.02,-.05],[.02,.02,.05],[.02,-.02,.05]],'faces':[[0,1,2]],'colors':[[.5]*3]*3}
        other=deepcopy(t);other['id']='A1';other['T_object_gripper'][0][3]+=.01
        second=deepcopy(t);second['object_id']='other'
        template={'id':'mesh','side':'left','vertices':[[0,0,0],[.1,0,0],[0,.1,0]],'faces':[[0,1,2]],'source':{}}
        config={'sampling':{'position_bounds_m':[[-.5,-.4,.8],[-.3,-.2,1]],'rpy_bounds_deg':[[-20,-20,-150],[20,20,-50]]}}
        with tempfile.TemporaryDirectory() as tmp, patch('r2handoversim.receivers.read_templates',return_value=(config,[template])):
            a=generate([t,other,second],'unused',tmp,2,27)
            b=generate([t,other,second],'unused',tmp,2,27)
        self.assertEqual(a,b)
        for i in range(2):
            self.assertEqual(a[i]['hand_mesh_world'],a[i+2]['hand_mesh_world'])
            self.assertEqual(a[i]['target_T_world_object'],a[i+2]['target_T_world_object'])
            self.assertNotEqual(a[i]['receiver']['T_world_hand'],a[i+4]['receiver']['T_world_hand'])
        self.assertNotEqual(a[0]['receiver']['T_world_hand'],a[1]['receiver']['T_world_hand'])

    def test_fixed_scene_delivery_cannot_move_hand(self):
        scene={'schema_version':'handover.scene.v1','object':{'id':'bottle','boxes':[box([0,0,0],[.02]*3)],
            'usage_regions':{'all':[box([0,0,0],[.02]*3)]}},'intent':{'human_region':'all'},
            'receiving_hand':{'center':[0,0,0],'boxes':[box([0,0,0],[.01]*3)]},
            'receiver_protocol':{'policy':'fixed_world','replan_in_isaac':True,'hand_collision':'mesh'},
            'target_T_world_object':pose([-.4,-.3,1]).tolist()}
        selection={'schema_version':'handover.selection.v1','object_id':'bottle','status':'ok','mode':'FS',
            'grasp_contract':{'width_policy':'asset_mesh_pad'},'selected':{'id':'g','T_object_gripper':np.eye(4).tolist(),'width_m':.04}}
        delivery={'schema_version':'handover.delivery.v1','units':'m','object_id':'bottle','grasp_id':'g',
            'T_world_gripper':scene['target_T_world_object'],'T_world_object':scene['target_T_world_object']}
        trial=from_selection(scene,selection,delivery=delivery)
        self.assertEqual(trial['receiver_protocol'],scene['receiver_protocol'])
        self.assertEqual(trial['gripper_opening_m'],.04)
        np.testing.assert_allclose(trial['palm_position_world'],[-.4,-.3,1])
        bad=deepcopy(delivery);bad['T_world_object']=pose([0,0,1]).tolist();bad['T_world_gripper']=bad['T_world_object']
        with self.assertRaisesRegex(ValueError,'cannot change'): from_selection(scene,selection,delivery=bad)

    def test_hull_reach_uses_volume_and_rotation(self):
        mesh={'vertices':[[x,y,z] for x in [-.1,.1] for y in [-.02,.02] for z in [-.02,.02]]}
        T=pose([1,2,3],[[0,-1,0],[1,0,0],[0,0,1]])
        self.assertTrue(hull_reach(mesh,T,[1,2,3],.001))
        self.assertTrue(hull_reach(mesh,T,[1,2.11,3],.02))
        self.assertFalse(hull_reach(mesh,T,[1.11,2,3],.02))

    def test_stability_failure_skips_search_and_plan_digest_detects_changes(self):
        t=load_demo('bottle');t.update(T_tcp_asset_tool=np.eye(4).tolist(),gripper_opening_m=.04,
            stability_width_m=.1,receiver_protocol={'planning_seed':1})
        with patch('r2handoversim.physx_planning.solve_pose') as ik:
            resolved=plan(t,lambda q: self.fail('Must not query collisions after stability failure'))
        ik.assert_not_called()
        self.assertEqual(resolved['planning']['reason'],'stability_gate')
        self.assertFalse(verified_plan(resolved))
        resolved['T_object_gripper'][0][3]+=.01
        with self.assertRaisesRegex(ValueError,'changed'): verified_plan(resolved)

    def test_conditioned_sampler_logs_rejections_before_any_method_selection(self):
        t=load_demo('bottle');t['asset_robot']={'usd':'local.usd','translation':[0,0,0]}
        t['object_mesh_object']={'vertices':[[0,0,0],[.02,.02,.05],[.02,-.02,.05]],'faces':[[0,1,2]],'colors':[[.5]*3]*3}
        t['T_tcp_asset_tool']=np.eye(4).tolist()
        template={'id':'mesh','side':'left','vertices':[[0,0,0],[.1,0,0],[0,.1,0]],'faces':[[0,1,2]],'source':{}}
        config={'sampling':{'position_bounds_m':[[-.5,-.4,.8],[-.3,-.2,1]],
            'rpy_bounds_deg':[[-20,-20,-150],[20,20,-50]],'require_reference_ik':True}}
        with tempfile.TemporaryDirectory() as tmp, patch('r2handoversim.receivers.read_templates',return_value=(config,[template])), patch('r2handoversim.planning.solve_pose',side_effect=[[],[[0]*6]]) as solve:
            rows=generate([t],'unused',tmp,1,27)
            bank=json.loads((Path(tmp)/'receivers.json').read_text())
        self.assertEqual(solve.call_count,2)
        self.assertEqual([p['accepted'] for p in bank['proposal_log']['bottle']],[False,True])
        self.assertEqual(rows[0]['receiver']['T_world_hand'],bank['proposal_log']['bottle'][1]['T_world_hand'])

    def test_feasibility_width_contract_cannot_be_changed_independently(self):
        from r2handoversim.evaluation import validate_trial
        t=load_demo('bottle')
        t['object_mesh_object']={'vertices':[[0,0,0],[.02,.02,.05],[.02,-.02,.05]],'faces':[[0,1,2]],'colors':[[.5]*3]*3}
        width=float(np.ptp(np.asarray(t['object_mesh_object']['vertices'])@np.asarray(t['T_object_gripper'])[:3,1]))
        t['gripper_opening_m']=.03
        t['grasp_contract']={'feasibility_width_policy':'object_projection'}
        t['method_selection']={'width_m':.03,'T_object_gripper':deepcopy(t['T_object_gripper']),'feasibility_width_m':width}
        validate_trial(t)
        t['method_selection']['feasibility_width_m']+=.01
        with self.assertRaisesRegex(ValueError,'feasibility width'): validate_trial(t)

    def test_fixed_experiment_defers_to_isaac_without_lifting_target(self):
        from r2handoversim.workflows import from_experiment
        target=pose([-.4,-.3,1]).tolist()
        scene={'schema_version':'handover.scene.v1','object':{'id':'bottle','boxes':[box([0,0,0],[.02]*3)],
            'usage_regions':{'all':[box([0,0,0],[.02]*3)]}},'intent':{'human_region':'all'},
            'receiving_hand':{'center':[0,0,0],'boxes':[box([0,0,0],[.01]*3)]},
            'receiver':{'id':'fixed'},'receiver_protocol':{'policy':'fixed_world','replan_in_isaac':True},
            'target_T_world_object':target}
        selection={'schema_version':'handover.selection.v1','object_id':'bottle','status':'ok','mode':'FS',
            'selected':{'id':'g','T_object_gripper':np.eye(4).tolist()}}
        manifest={'schema_version':'handover.experiment.v1','modes':['FS'],'objects':[{
            'object_id':'bottle','scene':'scene.json','selections':{'FS':'FS.json'}}]}
        with tempfile.TemporaryDirectory() as tmp, patch('r2handoversim.planning.plan_trial') as proxy:
            root=Path(tmp)
            for name,value in [('scene',scene),('FS',selection),('experiment',manifest)]:
                (root/f'{name}.json').write_text(json.dumps(value))
            rows,state=from_experiment(root/'experiment.json',root/'out')
        proxy.assert_not_called()
        self.assertEqual(rows[0]['target_T_world_object'],target)
        self.assertTrue(rows[0]['receiver_protocol']['replan_in_isaac'])
        self.assertEqual(rows[0]['experiment']['table_clearance_lift_m'],0.)

    def test_selected_method_cannot_be_resampled_after_selection(self):
        t=load_demo('bottle');t['method_selection']={'mode':'FS'}
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(ValueError,'BEFORE method selection'):
                generate([t],'unused',tmp)
