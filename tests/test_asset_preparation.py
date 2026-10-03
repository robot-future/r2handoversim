from copy import deepcopy
import unittest
from unittest.mock import patch
import numpy as np
from r2handoversim.asset_preparation import prepare, move_receiver
from r2handoversim.demos import load_demo
from r2handoversim.geometry import inverse, points, transform
from r2handoversim.robot import tcp


class Robot:
    def calibrate_opening(self,q,width):
        self.q=q
        return width,{'gap_m':width}
    def contact_distances(self,contacts): return [0.,0.]
    def tool_pose(self):
        offset=np.eye(4);offset[2,3]=.03
        return tcp(self.q)@offset


def fitted(mesh, grasp, max_opening):
    pose=transform(grasp);pose[:3,3]+=pose[:3,:3]@np.array([0.,0.,.018])
    return pose,{'status':'bilateral_surface_fit','width_m':.02,'T_object_gripper':pose.tolist(),
                 'contact_points_tool':[[0.,-.01,0.],[0.,.01,0.]]}


class PreparationTests(unittest.TestCase):
    def source(self):
        t=load_demo('hammer');t['asset_robot']={'usd':'example.usd','translation':[0,0,0]}
        t['object_mesh_object']={'vertices':[[0,0,0],[1,0,0],[0,1,0]],'faces':[[0,1,2]]}
        return t

    def test_resolved_scene_is_idempotent_and_never_refits(self):
        t=self.source();before=deepcopy(t)
        with patch('r2handoversim.grasp_fit.fit_grasp',side_effect=fitted) as fit:
            a,_=prepare(t,Robot());b,_=prepare(a,Robot())
            self.assertEqual(fit.call_count,1)
        self.assertEqual(a,b);self.assertEqual(t,before)
        changed=deepcopy(a);changed['object_mesh_object']['vertices'][0][0]=.1
        with self.assertRaisesRegex(ValueError,'assets changed'): prepare(changed,Robot())
        changed=deepcopy(a);changed['T_object_gripper'][0][3]+=.01
        with self.assertRaisesRegex(ValueError,'grasp changed'): prepare(changed,Robot())

    def test_hand_skeleton_and_delivery_stay_in_the_same_frame(self):
        t=self.source();G=transform(t['T_object_gripper'])
        t['delivery']={'keypoints_world':{'wrist':[1,2,3]},'shoulder_midpoint':[2,3,4],
                      'torso_center':[3,4,5],'hand_center_world':[1,1,1],'hand_direction_world':[1,0,0],
                      'facing_direction_world':[0,1,0], 'T_robot_gripper':G.tolist(),
                      'T_world_object':np.eye(4).tolist(),'T_world_gripper':G.tolist()}
        old=deepcopy(t);shift=np.eye(4);shift[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]];shift[:3,3]=[.2,.3,.4]
        move_receiver(t,shift)
        np.testing.assert_allclose(t['delivery']['keypoints_world']['wrist'],points(shift,[1,2,3]))
        np.testing.assert_allclose(t['palm_position_world'],points(shift,old['palm_position_world']))
        np.testing.assert_allclose(t['delivery']['hand_direction_world'],[0,1,0])
        np.testing.assert_allclose(t['delivery']['T_world_gripper'],shift@G)
        np.testing.assert_allclose(t['delivery']['T_robot_gripper'],shift@G)
        np.testing.assert_allclose(t['delivery']['hand_center_world'],points(shift,[1,1,1]))
        np.testing.assert_allclose(t['delivery']['facing_direction_world'],[-1,0,0])

    def test_reference_label_cannot_bypass_contact_fitting_for_a_narrow_grasp(self):
        t=self.source();t['replay_reference']={'assigned_outcome':'stability'}
        with patch('r2handoversim.grasp_fit.fit_grasp',side_effect=fitted) as fit:
            resolved,_=prepare(t,Robot())
            self.assertEqual(fit.call_count,1)
            self.assertEqual(resolved['asset_contact_fit']['status'],'bilateral_surface_fit')

    def test_fixed_receiver_is_never_retargeted_during_asset_preparation(self):
        t=self.source();t['receiver_protocol']={'policy':'fixed_world'}
        t['target_T_world_object']=np.eye(4).tolist()
        before=deepcopy(t)
        with patch('r2handoversim.grasp_fit.fit_grasp',side_effect=fitted):
            resolved,_=prepare(t,Robot())
        for key in ('hand_boxes_world','palm_position_world','palm_normal_world','target_T_world_object'):
            self.assertEqual(resolved[key],before[key])
        self.assertEqual(resolved['asset_preparation']['receiver_policy'],'fixed_world')
        np.testing.assert_allclose(resolved['target_T_world_gripper'],resolved['T_object_gripper'])

    def test_unprepared_method_candidate_is_not_refitted(self):
        t=self.source();t['method_selection']={'id':'g'}
        with patch('r2handoversim.grasp_fit.fit_grasp') as fit:
            with self.assertRaisesRegex(ValueError,'prepare-candidates'): prepare(t,Robot())
        fit.assert_not_called()
