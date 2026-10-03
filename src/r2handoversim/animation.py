"""Bake a self-contained replay USD from an exported final-scene snapshot."""
import numpy as np
from .evaluation import robot_geometry, trial_tcp
from .geometry import box_pose, inverse, moved


def bake(snapshot, destination, trial, asset_robot=None, opening=None):
    from pxr import Usd, UsdGeom, Gf
    stage = Usd.Stage.Open(str(snapshot))
    stage.SetTimeCodesPerSecond(1/trial["dt_s"])
    stage.SetFramesPerSecond(1/trial["dt_s"])
    stage.SetStartTimeCode(0)
    stage.SetEndTimeCode(len(trial["executed_joints"])-1)
    grasp_inverse = inverse(trial["T_object_gripper"])
    for frame, q in enumerate(trial["executed_joints"]):
        if asset_robot:
            asset_robot.update(q, opening)
            for path, source_op in asset_robot.ops.items():
                target_op = UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetOrderedXformOps()[0]
                target_op.Set(source_op.Get(), Usd.TimeCode(frame))
        cloud = stage.GetPrimAtPath("/World/Trial/ObjectMesh")
        if not cloud:
            cloud = stage.GetPrimAtPath("/World/Trial/ObjectCloud")
        if cloud:
            op = UsdGeom.Xformable(cloud).GetOrderedXformOps()[0]
            op.Set(Gf.Matrix4d(*(trial_tcp(trial, q)@grasp_inverse).T.reshape(-1).tolist()), Usd.TimeCode(frame))
        robot = robot_geometry(trial, q)
        objects = [moved(b, trial_tcp(trial, q) @ grasp_inverse) for b in trial["object_boxes"]]
        for group, boxes in (("Robot", robot), ("Object", objects)):
            for i, b in enumerate(boxes):
                prim = stage.GetPrimAtPath(f"/World/Trial/{group}/part_{i}")
                op = UsdGeom.Xformable(prim).GetOrderedXformOps()[0]
                matrix = box_pose(b)
                matrix[:3,:3] = matrix[:3,:3] @ np.diag(b["half_extents"])
                op.Set(Gf.Matrix4d(*matrix.T.reshape(-1).tolist()), Usd.TimeCode(frame))
    if not stage.Export(str(destination)):
        raise RuntimeError("USD animation export failed")
