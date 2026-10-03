"""Portable numerical pose IK and bidirectional RRT-Connect over box proxies.

Implements full-pose IK, seeded search and sampled proxy collision checks.
"""
from copy import deepcopy
import time
import numpy as np
from .geometry import box, intersects, inverse, moved, transform
from .evaluation import robot_geometry, validate_trial, trial_tcp
from .robot import HOME, joints, tcp


def solve_pose(target, initial=HOME, attempts=8, seed=0):
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
    target = transform(target)
    rng = np.random.default_rng(seed)
    def residual(q):
        actual = tcp(q)
        return np.r_[actual[:3, 3]-target[:3, 3],
                     .25*Rotation.from_matrix(target[:3, :3]@actual[:3, :3].T).as_rotvec()]
    solutions = []
    for start in [joints(initial), *rng.uniform(-np.pi, np.pi, (attempts-1, 6))]:
        result = least_squares(residual, start, bounds=(-2*np.pi, 2*np.pi), max_nfev=250,
                               ftol=1e-9, xtol=1e-9, gtol=1e-9)
        error = residual(result.x)
        if np.linalg.norm(error[:3]) <= .001 and np.linalg.norm(error[3:])/.25 <= .005:
            solutions.append(result.x)
    return sorted(solutions, key=lambda q: np.linalg.norm(q-initial))


def _overlap(a, b):
    # Conservative sphere broad phase; the narrow phase is oriented-box SAT.
    distance = np.linalg.norm(np.asarray(a["center"])-b["center"])
    if distance > np.linalg.norm(a["half_extents"])+np.linalg.norm(b["half_extents"]):
        return False
    return intersects(a, b)


def configuration_clear(trial, q):
    if np.any(np.abs(joints(q)) > 2*np.pi):
        return False
    robot = robot_geometry(trial, q)
    attached = [moved(b, trial_tcp(trial, q)@inverse(trial["T_object_gripper"])) for b in trial["object_boxes"]]
    obstacles = trial.get("obstacle_boxes_world", [])
    hand = trial["hand_boxes_world"]
    if any(_overlap(a, b) for a in robot+attached for b in hand+obstacles):
        return False
    # Adjacent links and their two-hop neighbours overlap by proxy construction.
    arm = robot[:7]
    if any(_overlap(arm[i], arm[j]) for i in range(len(arm)) for j in range(i+3, len(arm))):
        return False
    # Exclude the distal wrist/tool links that intentionally meet the held object.
    if any(_overlap(a, b) for a in attached+robot[7:] for b in arm[:4]):
        return False
    return True


def edge_samples(a, b, resolution=.06):
    n = max(1, int(np.ceil(np.max(np.abs(np.asarray(b)-a))/resolution)))
    return np.linspace(a, b, n+1)


def rrt_connect(start, goal, clear, rng, iterations=600, step=.25, resolution=.06):
    """Return a densely checked path, including both endpoints, or None."""
    start, goal = np.asarray(start), np.asarray(goal)
    def edge(a, b):
        return all(clear(q) for q in edge_samples(a, b, resolution))
    if not clear(start) or not clear(goal):
        return None
    if edge(start, goal):
        return edge_samples(start, goal, resolution).tolist()
    trees = [([start], [-1]), ([goal], [-1])]
    def extend(tree, target):
        nodes, parents = tree
        near = int(np.argmin([np.linalg.norm(q-target) for q in nodes]))
        delta = target-nodes[near]
        dist = np.linalg.norm(delta)
        if dist < 1e-10:
            return near, True
        q = nodes[near] + delta*min(1., step/dist)
        if not edge(nodes[near], q):
            return None, False
        nodes.append(q); parents.append(near)
        return len(nodes)-1, dist <= step
    def branch(tree, index):
        path = []
        while index >= 0:
            path.append(tree[0][index]); index = tree[1][index]
        return path[::-1]
    for iteration in range(iterations):
        active = iteration % 2
        a, b = trees[active], trees[1-active]
        target = b[0][0] if rng.random() < .15 else rng.uniform(-2*np.pi, 2*np.pi, len(start))
        ia, _ = extend(a, target)
        if ia is None:
            continue
        while True:
            ib, reached = extend(b, a[0][ia])
            if ib is None:
                break
            if reached:
                path = branch(a, ia) + branch(b, ib)[::-1][1:]
                if active == 1:
                    path = path[::-1]
                dense = [path[0].tolist()]
                for left, right in zip(path[:-1], path[1:]):
                    dense.extend(edge_samples(left, right, resolution)[1:].tolist())
                return dense
    return None


def plan_trial(trial, seed=0, iterations=600, max_joint_speed=.6, resolution=.06):
    """Returns an auditable failed trial if IK or search fails; no invented path."""
    if trial.get('receiver_protocol',{}).get('planner')=='isaacsim_physx_rrt_connect':
        raise ValueError('Fixed receiver mesh scenes must be planned by demo in Isaac Sim, not the proxy planner')
    validate_trial(trial)
    if iterations < 1 or not np.isfinite([max_joint_speed, resolution]).all() or max_joint_speed <= 0 or resolution <= 0:
        raise ValueError("Planner iteration count, speed and resolution must be positive")
    result = deepcopy(trial)
    result.setdefault("obstacle_boxes_world", [box([-.35, 0, .70], [.65, .55, .035], label="table")])
    started = time.perf_counter()
    start = joints(trial["planned_joints"][0])
    offset = transform(trial.get("T_tcp_asset_tool", np.eye(4)))
    candidates = solve_pose(transform(trial["target_T_world_gripper"]) @ inverse(offset), start, seed=seed)
    rng, path = np.random.default_rng(seed), None
    for goal in candidates:
        path = rrt_connect(start, goal, lambda q: configuration_clear(result, q), rng,
                           iterations=iterations, resolution=resolution)
        if path is not None:
            break
    result["planning"] = {"status": "succeeded" if path is not None else "failed",
        "planner": "release numerical Jacobian IK + RRT-Connect", "seed": seed,
        "ik_solutions": len(candidates), "edge_resolution_rad": resolution,
        "max_joint_speed_rad_s": max_joint_speed, "time_s": time.perf_counter()-started,
        "reason": None if path is not None else ("search_or_collision_failure" if candidates else "ik_failure"),
        "collision_scope": "sampled robot/object/hand/obstacle boxes; nonadjacent arm proxies; distal grasp contacts excluded"}
    # Densify linearly to the simulator timestep; never smooth around unchecked corners.
    executed = [start.tolist()]
    if path is not None:
        for a, b in zip(path[:-1], path[1:]):
            step = min(resolution, max_joint_speed*trial["dt_s"])
            executed.extend(edge_samples(np.asarray(a), np.asarray(b), step)[1:].tolist())
    result["planned_joints"] = deepcopy(executed)
    result["executed_joints"] = executed
    return result
