"""Standalone demo fixtures and conversion from Intent-Handover scene/selection."""
from copy import deepcopy
from importlib.resources import files
import json
import numpy as np
from .geometry import box, inverse, moved, points, pose, transform, unit
from .robot import GOAL, HOME, interpolate, solve_position, tcp

NAMES = ("hammer", "screwdriver", "bottle")
VARIANTS = ("intent_aware", "region_agnostic", "execution_deviation", "missed_delivery")


def load_demo(name, variant="intent_aware"):
    if name not in NAMES or variant not in VARIANTS:
        raise ValueError("Unknown demo object or variant")
    return json.loads(files(__package__).joinpath("assets", f"{name}_{variant}.json").read_text())


def from_selection(scene, selection, steps=90, variant="intent_aware", delivery=None):
    if scene.get("schema_version") != "handover.scene.v1" or selection.get("schema_version") != "handover.selection.v1":
        raise ValueError("Expected Intent-Handover v1 scene and selection")
    if selection.get("status") != "ok" or not selection.get("selected"):
        raise ValueError("Cannot replay a selection without a feasible grasp")
    if selection["object_id"] != scene["object"]["id"]:
        raise ValueError("Scene and selection object ids differ")
    grasp = transform(selection["selected"]["T_object_gripper"])
    goal = tcp(GOAL)
    fixed_scene = scene.get('receiver_protocol',{}).get('policy') == 'fixed_world'
    if fixed_scene:
        goal = transform(scene['target_T_world_object']) @ grasp
    if delivery is not None:
        if (delivery.get("schema_version") != "handover.delivery.v1" or delivery.get("units") != "m"
                or delivery["object_id"] != scene["object"]["id"]
                or delivery["grasp_id"] != selection["selected"]["id"]):
            raise ValueError("Delivery target must match this object and selected grasp")
        goal = transform(delivery["T_world_gripper"])
        if not np.allclose(goal@inverse(grasp), transform(delivery["T_world_object"]), atol=1e-6):
            raise ValueError("Delivery object and gripper transforms are inconsistent")
        if fixed_scene and not np.allclose(delivery['T_world_object'],scene['target_T_world_object'],atol=1e-9):
            raise ValueError('Delivery cannot change a fixed receiver scene object target')
    world_object = goal @ inverse(grasp)
    region = scene["intent"]["human_region"]
    # Receiving hand in object coordinates; bundled demos use procedural proxies.
    palm_object = np.array(scene["receiving_hand"]["center"], dtype=float)
    palm_world = points(world_object, palm_object)
    normal_world = world_object[:3, :3] @ unit(scene["receiving_hand"].get("palm_normal", [1., 0., 0.]))
    hand_boxes = [moved(b, world_object) for b in scene["receiving_hand"]["boxes"]]
    planned = interpolate(HOME, GOAL, steps)
    executed = deepcopy(planned)
    if variant == "execution_deviation":
        # Unplanned deviation deliberately takes the TCP through the palm.
        bad = solve_position(hand_boxes[0]["center"])
        executed = interpolate(HOME, bad, steps//2) + interpolate(bad, GOAL, steps//2)[1:]
    elif variant == "missed_delivery":
        # Deliberately stop at home instead of reaching the planned goal.
        executed = [HOME.tolist()] * steps
    trial = {"schema_version": "handover.trial.v1", "units": "m",
            "id": f"{scene['object']['id']}_{variant}", "object_id": scene["object"]["id"],
            "variant": variant, "split": scene.get("evaluation_split", "S0" if scene["object"]["id"] == "bottle" else "S1"),
            "provenance": scene.get("provenance", "User supplied scene"),
            "T_object_gripper": grasp.tolist(), "target_T_world_gripper": goal.tolist(),
            "object_boxes": deepcopy(scene["object"]["boxes"]),
            "usage_boxes": deepcopy(scene["object"]["usage_regions"][region]),
            "hand_boxes_world": hand_boxes, "palm_position_world": palm_world.tolist(),
            "palm_normal_world": normal_world.tolist(), "reach_offset_m": .12,
            "reach_radius_m": .10, "max_opening_m": scene.get('gripper', {}).get('max_opening_m', .085), "dt_s": 1/60,
            "planned_joints": planned, "executed_joints": executed}
    if 'evaluation_split_provenance' in scene:
        trial['evaluation_split_provenance']=deepcopy(scene['evaluation_split_provenance'])
    if 'grasp_contract' in selection:
        trial['grasp_contract'] = deepcopy(selection['grasp_contract'])
        trial['method_selection'] = {**deepcopy(selection['selected']), 'mode': selection.get('mode')}
        trial['gripper_opening_m'] = float(selection['selected']['width_m'])
        trial['gripper_opening_source'] = selection['selected'].get('width_source', 'method_selection')
    if 'mesh' in scene['object']:
        mesh = deepcopy(scene['object']['mesh'])
        mesh.setdefault('colors', [[.72,.78,.82]] * len(mesh['vertices']))
        trial['object_mesh_object'] = mesh
        if 'geometry_contract' in scene.get('gripper',{}):
            trial['asset_robot'] = deepcopy(scene['gripper']['geometry_contract']['robot'])
    if 'receiver' in scene:
        trial['receiver'] = deepcopy(scene['receiver'])
    if fixed_scene:
        trial['receiver_protocol'] = deepcopy(scene['receiver_protocol'])
        trial['target_T_world_object'] = deepcopy(scene['target_T_world_object'])
        trial['planned_joints'] = [HOME.tolist()]
        trial['executed_joints'] = [HOME.tolist()]
        if 'geometry_contract' in scene.get('gripper',{}):
            trial['asset_robot'] = deepcopy(scene['gripper']['geometry_contract']['robot'])
    if "surface_points" in scene["object"]:
        trial["object_points_object"] = deepcopy(scene["object"]["surface_points"])
        trial["source_data"] = deepcopy(scene.get("source_data", {}))
        trial["annotation_status"] = scene.get("annotation_status", "user supplied")
    if "mesh" in scene["receiving_hand"]:
        mesh = scene["receiving_hand"]["mesh"]
        trial["hand_mesh_world"] = {"vertices": points(world_object, mesh["vertices"]).tolist(),
                                    "faces": deepcopy(mesh["faces"])}
    if delivery is not None:
        # A target pose is not a solved trajectory. The caller must run plan_trial.
        trial["planned_joints"] = [HOME.tolist()]
        trial["executed_joints"] = [HOME.tolist()]
        trial["delivery"] = deepcopy(delivery)
        if not fixed_scene:
            trial['receiver_protocol'] = {'policy':'fixed_world', 'replan_in_isaac':False}
        trial['target_T_world_object'] = deepcopy(delivery['T_world_object'])
        if 'asset_robot' in trial and 'hand_mesh_world' not in trial:
            raise ValueError('Original-asset delivery requires receiving_hand.mesh; use receiver-scenes or a decoded MANO scene')
        if 'asset_robot' in trial and 'hand_mesh_world' in trial:
            trial['receiver_protocol'].update(replan_in_isaac=True,hand_collision='mesh',
                object_collision='convexHull',planner='isaacsim_physx_rrt_connect')
    return trial
