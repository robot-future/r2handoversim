"""RRT-Connect with original USD robot colliders, object hull and hand mesh.

The scene query is supplied by the live Isaac Sim stage; no MoveIt is claimed.
"""
from copy import deepcopy
import hashlib
import json
import time
import numpy as np
from .geometry import inverse, transform
from .planning import solve_pose, rrt_connect, edge_samples
from .robot import HOME


def scene_digest(trial):
    keys=['T_object_gripper','T_tcp_asset_tool','target_T_world_gripper','gripper_opening_m',
          'asset_robot','object_mesh_object','hand_mesh_world','obstacle_boxes_world',
          'planned_joints','max_opening_m']
    value={k:trial[k] for k in keys if k in trial}
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def plan(trial, clear, iterations=200, speed=.6, resolution=.06):
    started=time.perf_counter();result=deepcopy(trial)
    seed=trial['receiver_protocol'].get('planning_seed',0)
    start=np.asarray(trial['planned_joints'][0]); offset=transform(trial['T_tcp_asset_tool'])
    stable=float(trial.get('stability_width_m',trial['gripper_opening_m'])) <= trial['max_opening_m']+1e-9
    candidates=solve_pose(transform(trial['target_T_world_gripper'])@inverse(offset),initial=start,seed=seed) if stable else []
    checked=0; reasons={}; cache={}
    def check(q):
        nonlocal checked
        key=tuple(np.round(q,10))
        if key not in cache:
            ok,reason=clear(q);checked+=1;cache[key]=bool(ok)
            if not ok: reasons[reason]=reasons.get(reason,0)+1
        return cache[key]
    path=None
    if stable and check(start):
        for goal in candidates:
            path=rrt_connect(start,goal,check,np.random.default_rng(seed),iterations=iterations,resolution=resolution)
            if path is not None: break
    dense=[start.tolist()]
    if path is not None:
        for a,b in zip(path[:-1],path[1:]):
            dense.extend(edge_samples(np.asarray(a),np.asarray(b),min(resolution,speed*trial['dt_s']))[1:].tolist())
        # Validate the exact executed states as well as the search edges.
        valid=[check(q) for q in dense]
        if not all(valid): path=None;dense=[start.tolist()]
    result['planned_joints']=deepcopy(dense);result['executed_joints']=dense
    result['planning']={'status':'succeeded' if path is not None else 'failed',
        'planner':'Numerical pose IK + RRT-Connect with Isaac Sim PhysX scene queries',
        'collision_backend':'isaacsim_physx_mesh','seed':seed,'time_s':time.perf_counter()-started if stable else 0.,
        'ik_solutions':len(candidates),'checked_configurations':checked,'rejected_contacts':reasons,
        'edge_resolution_rad':resolution,'max_joint_speed_rad_s':speed,
        'reason':None if path is not None else ('stability_gate' if not stable else 'ik_failure' if not candidates else 'collision_or_search_failure'),
        'collision_scope':'Original USD robot colliders; object convex hull; static hand triangles; table/obstacles; nonadjacent robot links',
        'allowed_contacts':'Same/adjacent robot links and gripper internal links; held object versus gripper/distal wrist; shoulder mounting surface',
        'path_collision_free':path is not None,'validated_frames':len(dense) if path is not None else 0}
    result['planning']['scene_sha256']=scene_digest(result)
    return result


def verified_plan(trial):
    planning=trial['planning']
    if planning.get('scene_sha256') != scene_digest(trial):
        raise ValueError('PhysX planned scene or trajectory changed; replan in Isaac Sim')
    return (planning['status']=='succeeded' and planning.get('path_collision_free') is True
            and planning.get('validated_frames')==len(trial['planned_joints']))


def link_group(path):
    path=str(path)
    if path.startswith('/World/Trial/Hand/'): return 'hand'
    if path.startswith('/World/Trial/ObjectMesh'): return 'object'
    if path.startswith('/World/Trial/Obstacles/') or path.startswith('/World/Table'): return 'environment'
    if path.startswith('/World/AssetRobot/world/'): return 'environment'
    for i,name in enumerate(['shoulder','upper_arm','forearm','wrist_1','wrist_2','wrist_3']):
        if f'/ur5e_{name}_link/' in path: return i
    if path.startswith('/World/AssetRobot/'): return 6
    return 'ignore'


def forbidden(a,b):
    """Explicit allowed-contact groups for the supported UR5e/Robotiq assembly."""
    if a==b or 'ignore' in (a,b): return False
    if a=='object' or b=='object':
        other=b if a=='object' else a
        return other in ('hand','environment') or isinstance(other,int) and other<=2
    if isinstance(a,int) and isinstance(b,int): return abs(a-b)>2
    if 'hand' in (a,b): return isinstance(a,int) or isinstance(b,int)
    if 'environment' in (a,b):
        other=b if a=='environment' else a
        return isinstance(other,int) and other!=0
    return False
