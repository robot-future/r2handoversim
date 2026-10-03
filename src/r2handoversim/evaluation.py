"""Evaluate replay traces against the five criteria with explicit proxy scope."""
from collections import Counter
import re
import numpy as np
from .geometry import (box_pose, gripper_boxes, intersects, inverse, moved, points,
                       pose, projected_width, sphere_intersects, transform, unit, vector)
from .robot import arm_boxes, joints, tcp

ORDER = ("stability", "plan", "reach", "affordance", "safe")


def trial_tcp(trial, q):
    return tcp(q) @ transform(trial.get("T_tcp_asset_tool", np.eye(4)))


def validate_trial(trial):
    if not isinstance(trial, dict):
        raise ValueError('Each trial must be a JSON object')
    if trial.get("schema_version") != "handover.trial.v1" or trial.get("units") != "m":
        raise ValueError("Expected handover.trial.v1 with units=m")
    if not isinstance(trial.get("id"), str) or not re.fullmatch(r"[a-zA-Z0-9_-]+", trial["id"]):
        raise ValueError("Trial id must contain only letters, digits, underscores and hyphens")
    for field in ('object_id', 'variant'):
        if not isinstance(trial.get(field), str) or not trial[field]:
            raise ValueError(f'{field} must be a nonempty string')
    if 'gripper_opening_m' in trial:
        width = trial['gripper_opening_m']
        if not isinstance(width, (float, int)) or not np.isfinite(width) or width <= 0:
            raise ValueError('gripper_opening_m must be positive and finite')
    if 'method_selection' in trial:
        selected = trial['method_selection']
        if not np.isclose(trial['gripper_opening_m'], selected['width_m'], atol=1e-10, rtol=0):
            raise ValueError('Method opening differs from selected grasp width')
        if not trial.get('method_geometry_invalidated') and not np.allclose(
                trial['T_object_gripper'], selected['T_object_gripper'], atol=1e-10, rtol=0):
            raise ValueError('Method grasp changed without explicit invalidation')
    if 'asset_robot' in trial:
        config = trial['asset_robot']
        if not isinstance(config, dict) or not isinstance(config.get('usd'), str) or not config['usd']:
            raise ValueError('asset_robot.usd must reference a local USD file')
        vector(config['translation'])
        if 'object_mesh_object' not in trial:
            raise ValueError('Original robot replay requires object_mesh_object')
    if 'receiver' in trial:
        receiver=trial['receiver'];hand_pose=transform(receiver['T_world_hand'])
        if receiver.get('static_world') is not True or receiver.get('side') not in ('left','right'):
            raise ValueError('Receiver must declare side and static_world')
        if not np.allclose(hand_pose[:3,3],trial['palm_position_world'],atol=1e-8):
            raise ValueError('Receiver palm differs from the declared hand pose')
        if not np.allclose(hand_pose[:3,2],unit(trial['palm_normal_world']),atol=1e-8):
            raise ValueError('Receiver normal differs from the declared hand pose')
        if not np.allclose(transform(trial['target_T_world_object'])@transform(trial['T_object_gripper']),
                           trial['target_T_world_gripper'],atol=1e-8):
            raise ValueError('Fixed object target and grasp differ from tool target')
    if (trial.get('grasp_contract',{}).get('feasibility_width_policy')=='object_projection'
            and 'method_selection' in trial):
        measured=float(np.ptp(np.asarray(trial['object_mesh_object']['vertices']) @ transform(trial['T_object_gripper'])[:3,1]))
        if not np.isclose(measured,trial['method_selection']['feasibility_width_m'],atol=1e-8,rtol=0):
            raise ValueError('Method feasibility width differs from the original object projection')
    if "hand_mesh_world" in trial:
        mesh = trial["hand_mesh_world"]
        vertices, faces = np.asarray(mesh["vertices"]), np.asarray(mesh["faces"])
        if (vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 3
                or not np.isfinite(vertices).all() or faces.ndim != 2 or faces.shape[1] != 3
                or not len(faces) or not np.issubdtype(faces.dtype, np.integer)
                or np.any(faces < 0) or np.any(faces >= len(vertices))):
            raise ValueError("Invalid hand triangle mesh")
    if "T_tcp_asset_tool" in trial:
        transform(trial["T_tcp_asset_tool"])
    if "object_mesh_object" in trial:
        mesh = trial["object_mesh_object"]
        vertices, faces, colors = (np.asarray(mesh[k]) for k in ("vertices", "faces", "colors"))
        if (vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 3
                or not np.isfinite(vertices).all() or faces.ndim != 2 or faces.shape[1] != 3
                or not len(faces) or not np.issubdtype(faces.dtype, np.integer)
                or np.any(faces < 0) or np.any(faces >= len(vertices))
                or colors.shape != vertices.shape or not np.isfinite(colors).all()
                or np.any(colors < 0) or np.any(colors > 1)):
            raise ValueError("Invalid object triangle mesh or vertex colors")
    if trial["split"] not in ("S0", "S1"):
        raise ValueError("Split must be S0 or S1")
    if "object_points_object" in trial:
        cloud = np.asarray(trial["object_points_object"])
        if cloud.ndim != 2 or cloud.shape[1] != 3 or len(cloud) < 4 or not np.isfinite(cloud).all():
            raise ValueError("Object point cloud must contain finite XYZ points")
    transform(trial["T_object_gripper"])
    transform(trial["target_T_world_gripper"])
    if not trial["planned_joints"] or not trial["executed_joints"]:
        raise ValueError("Both planned and executed trajectories must be nonempty")
    for q in trial["planned_joints"] + trial["executed_joints"]:
        joints(q)
    for b in trial["object_boxes"] + trial["usage_boxes"] + trial["hand_boxes_world"]:
        box_pose(b)
    for b in trial.get("obstacle_boxes_world", []):
        box_pose(b)
    if "planning" in trial:
        planning = trial["planning"]
        if planning["status"] not in ("succeeded", "failed") or not np.isfinite(planning["time_s"]) or planning["time_s"] < 0:
            raise ValueError("Invalid planning status or time")
    if not trial["object_boxes"] or not trial["hand_boxes_world"]:
        raise ValueError("Object and receiving hand geometries cannot be empty")
    if trial["split"] == "S1" and not trial["usage_boxes"]:
        raise ValueError("S1 requires intended usage geometry")
    for k in ("max_opening_m", "dt_s", "reach_radius_m"):
        if not np.isfinite(trial[k]) or trial[k] <= 0:
            raise ValueError(f"{k} must be positive")
    vector(trial["palm_position_world"])
    unit(trial["palm_normal_world"])
    if not np.isfinite(trial["reach_offset_m"]):
        raise ValueError("Reach offset must be finite")


def grasp_width(trial):
    if 'gripper_opening_m' in trial:
        return float(trial['gripper_opening_m'])
    fit = trial.get('asset_contact_fit', {})
    if fit.get('status') == 'bilateral_surface_fit':
        return float(fit['width_m'])
    return projected_width(trial['object_boxes'], transform(trial['T_object_gripper'])[:3, 1])


def robot_geometry(trial, q, width=None):
    if width is None:
        width = grasp_width(trial)
    return arm_boxes(q) + [moved(b, trial_tcp(trial, q)) for b in gripper_boxes(min(width, trial["max_opening_m"]))]


def hand_contact(trial, q, width):
    return any(intersects(a, b) for a in robot_geometry(trial, q, width) for b in trial["hand_boxes_world"])


def evaluate(trial, physics_contacts=None):
    validate_trial(trial)
    grasp = transform(trial["T_object_gripper"])
    width = grasp_width(trial)
    stability_width = float(trial.get('stability_width_m', width))
    stable = stability_width <= trial["max_opening_m"] + 1e-9
    planned, executed = trial["planned_joints"], trial["executed_joints"]
    target = transform(trial["target_T_world_gripper"])
    end = trial_tcp(trial, planned[-1])
    endpoint_ok = np.linalg.norm(end[:3, 3] - target[:3, 3]) <= .005 and np.allclose(end[:3, :3], target[:3, :3], atol=.01)
    limits_ok = all(np.all(np.abs(joints(q)) <= 2*np.pi) for q in planned)
    # A sampled path check, no planner search, self-collision or obstacle checking.
    mesh_plan = trial.get('planning',{}).get('collision_backend') == 'isaacsim_physx_mesh'
    plan_contact = False if mesh_plan else any(hand_contact(trial, q, width) for q in planned)
    plan = bool(endpoint_ok and limits_ok and not plan_contact)
    plan_scope = "Provided joint path: joint limits, endpoint and sampled hand clearance only"
    if "planning" in trial:
        from .planning import configuration_clear, edge_samples
        # Recheck edges too: editing or sparsifying a planned trace cannot skip an obstacle.
        clear = trial["planning"]["status"] == "succeeded"
        mesh_plan = trial['planning'].get('collision_backend') == 'isaacsim_physx_mesh'
        if mesh_plan:
            from .physx_planning import verified_plan
            clear = verified_plan(trial)
        elif clear:
            clear = configuration_clear(trial, planned[0]) and all(
                configuration_clear(trial, q) for a, b in zip(planned[:-1], planned[1:])
                for q in edge_samples(np.asarray(a), np.asarray(b))[1:])
        plan = bool((endpoint_ok and limits_ok if mesh_plan else plan) and clear)
        plan_scope = "Numerical pose IK + RRT-Connect; robot/object/hand/obstacle box checks, sampled edges and nonadjacent arm checks"
        if mesh_plan: plan_scope = trial['planning']['collision_scope']
    final_world_object = trial_tcp(trial, executed[-1]) @ inverse(grasp)
    delivered = [moved(b, final_world_object) for b in trial["object_boxes"]]
    sphere_center = vector(trial["palm_position_world"]) + trial["reach_offset_m"]*unit(trial["palm_normal_world"])
    reach = any(sphere_intersects(b, sphere_center, trial["reach_radius_m"]) for b in delivered)
    reach_source = 'oriented object boxes'
    if trial.get('receiver_protocol',{}).get('object_collision') == 'convexHull':
        from .mesh_metrics import hull_reach
        reach = hull_reach(trial['object_mesh_object'],final_world_object,sphere_center,trial['reach_radius_m'])
        reach_source = 'original object convex hull versus palm-normal sphere'
    fingers = [moved(b, grasp) for b in gripper_boxes(min(width, trial["max_opening_m"])) if b["label"] == "finger"]
    affordance = (not any(intersects(a, b) for a in fingers for b in trial["usage_boxes"])) if trial["split"] == "S1" else None
    affordance_source = 'finger and usage-region box proxies' if trial['split']=='S1' else None
    if trial['split']=='S1' and 'affordance_observation' in trial:
        from .mesh_metrics import affordance_digest
        observed=trial['affordance_observation']
        if observed['geometry_sha256']!=affordance_digest(trial) or not isinstance(observed['clear'],bool):
            raise ValueError('USD finger/usage-region observation does not match the replay geometry')
        affordance=observed['clear'];affordance_source=observed['source']
    if physics_contacts is None:
        contact_indices = [i for i, q in enumerate(executed) if hand_contact(trial, q, width)]
        safe_source = "sampled oriented-box intersections"
    else:
        if len(physics_contacts) != len(executed):
            raise ValueError("Physics contact trace must cover every executed frame")
        if not all(isinstance(hit, (bool, np.bool_)) for hit in physics_contacts):
            raise ValueError("Physics contacts must be boolean observations")
        contact_indices = [i for i, hit in enumerate(physics_contacts) if hit]
        safe_source = "Isaac Sim PhysX overlap queries at every replay frame"
    values = {"stability": bool(stable), "plan": plan, "reach": bool(reach),
              "affordance": affordance, "safe": not contact_indices}
    failure = next((k for k in ORDER if values[k] is False), None)
    return {"trial_id": trial["id"], "object_id": trial["object_id"], "split": trial["split"],
            "variant": trial["variant"], "evaluation_split_provenance": trial.get("evaluation_split_provenance"), "metrics": values, "success": failure is None,
            "first_failure": failure, "width_m": width, "contact_frames": contact_indices,
            "stability_width_m": stability_width,
            "stability_width_rule": trial.get('stability_width_rule', trial.get('gripper_opening_source', 'legacy_projection')),
            "grasp_contract": trial.get('grasp_contract'), "method_selection": trial.get('method_selection'),
            "receiver": trial.get('receiver'), "receiver_protocol": trial.get('receiver_protocol'),
            "reach_source":reach_source,"affordance_source":affordance_source,
            "safe_source": safe_source, "trajectory_duration_s": (len(executed)-1)*trial["dt_s"],
            "planning_time_s": trial.get("planning", {}).get("time_s"), "execution_wall_time_s": None,
            "execution_time_s": (len(executed)-1)*trial["dt_s"],
            "total_time_s": (trial["planning"]["time_s"]+(len(executed)-1)*trial["dt_s"]) if "planning" in trial else None,
            "scope": trial.get("provenance", "Procedural geometry and UR5e joint replay"),
            "source_data": trial.get("source_data"), "annotation_status": trial.get("annotation_status"),
            "plan_scope": plan_scope, "replay_reference": trial.get("replay_reference")}


def summarize(results):
    groups = {}
    for split in ("S0", "S1"):
        rows = [r for r in results if r["split"] == split]
        if not rows:
            continue
        counts = Counter(r["first_failure"] for r in rows if not r["success"])
        groups[split] = {"trials": len(rows), "success_rate": sum(r["success"] for r in rows)/len(rows),
                         "failure_rates": {k: (None if k == "affordance" and split == "S0" else counts[k]/len(rows)) for k in ORDER}}
    return {"schema_version": "handover.summary.v1", "scope": "Evaluated demo results", "splits": groups}
