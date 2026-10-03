"""Geometric metrics for original object hulls and recorded USD finger queries."""
import hashlib
import json
import numpy as np
from .geometry import inverse, points


def hull_reach(mesh, world_object, center, radius):
    from scipy.spatial import ConvexHull
    from trimesh.triangles import closest_point
    vertices=np.asarray(mesh['vertices'],dtype=float)
    hull=ConvexHull(vertices)
    local=points(inverse(world_object),center)
    if np.all(hull.equations[:,:3]@local+hull.equations[:,3]<=1e-9): return True
    triangles=vertices[hull.simplices]
    closest=closest_point(triangles,np.repeat(local[None],len(triangles),axis=0))
    return bool(np.min(np.linalg.norm(closest-local,axis=1))<=radius+1e-9)


def affordance_digest(trial):
    fields=['asset_robot','T_object_gripper','T_tcp_asset_tool','gripper_opening_m','usage_boxes']
    value={k:trial.get(k) for k in fields}
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
