import unittest
import numpy as np
from r2handoversim.camera import visibility, choose


class CameraTests(unittest.TestCase):
    def test_parallel_and_blocked_rays(self):
        bounds=[[[.4,-.1,-.1],[.6,.1,.1]]]
        self.assertEqual(visibility([1,0,0],[[0,0,0]],bounds),0.)
        self.assertEqual(visibility([0,1,0],[[0,0,0]],bounds),1.)
        self.assertEqual(visibility([0,1,0],[[0,0,0]],[]),1.)
        self.assertEqual(visibility([0,1,0],[[0,0,0]],[[[-.1,.4,-.1],[.1,.6,.1]]]),0.)

    def test_review_camera_avoids_forearm_occlusion_without_changing_scene(self):
        hand=np.array([[0,0,0],[0,.02,0],[0,-.02,0]])
        target=np.array([0.,0.,0.]);before=hand.copy()
        bounds=[[[[.1,-.3,-.2],[.9,.3,.9]]]]
        eye,report=choose(hand,target,bounds)
        self.assertGreater(report['sampled_pose_visibility'][0],.9)
        self.assertLess(eye[0],.2)
        np.testing.assert_array_equal(hand,before)
        np.testing.assert_array_equal(target,[0,0,0])
