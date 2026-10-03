import unittest
import numpy as np
from r2handoversim.grasp_fit import fit_grasp


def cuboid(width, x=0):
    vertices=[[x+a*0.01,b*width/2,c*.06] for a in (-1,1) for b in (-1,1) for c in (0,1)]
    faces=[[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],[2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]]
    return {'vertices':vertices,'faces':faces}


class GraspFitTests(unittest.TestCase):
    def test_surface_tcp_is_inserted_and_local_width_matches_contacts(self):
        fitted, audit=fit_grasp(cuboid(.025),np.eye(4))
        self.assertGreater(fitted[2,3],.01)
        self.assertAlmostEqual(audit['width_m'],.025)
        a,b=np.asarray(audit['contact_points_tool'])
        self.assertAlmostEqual(a[1],-.0125)
        self.assertAlmostEqual(b[1],.0125)
        self.assertTrue(np.all(np.abs([a[0],b[0]])<=.010001))

    def test_empty_approach_is_shifted_to_actual_mesh(self):
        fitted,audit=fit_grasp(cuboid(.02,x=.04),np.eye(4))
        self.assertGreater(fitted[0,3],.02)
        self.assertAlmostEqual(audit['width_m'],.02)

    def test_too_wide_object_is_not_attached_as_a_valid_grasp(self):
        with self.assertRaisesRegex(ValueError,'No opposing'):
            fit_grasp(cuboid(.12),np.eye(4))
