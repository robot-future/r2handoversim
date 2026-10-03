"""Prepare a local-asset replay once, retaining its coordinate changes."""
from copy import deepcopy
import hashlib
import numpy as np
from .geometry import inverse, moved, points, transform, projected_width
from .robot import tcp


def mesh_digest(mesh):
    digest = hashlib.sha256()
    for key, dtype in [('vertices', '<f8'), ('faces', '<i8')]:
        array = np.asarray(mesh[key], dtype=dtype)
        digest.update(str(array.shape).encode())
        digest.update(array.tobytes())
    return digest.hexdigest()


def move_receiver(trial, shift):
    """Apply one world transform to the hand, skeleton and delivery annotations."""
    trial['hand_boxes_world'] = [moved(b, shift) for b in trial['hand_boxes_world']]
    trial['palm_position_world'] = points(shift, trial['palm_position_world']).tolist()
    trial['palm_normal_world'] = (shift[:3,:3] @ trial['palm_normal_world']).tolist()
    if 'hand_mesh_world' in trial:
        mesh = trial['hand_mesh_world']
        mesh['vertices'] = points(shift, mesh['vertices']).tolist()
    if 'delivery' in trial:
        delivery = trial['delivery']
        robot_world = (transform(delivery['T_robot_gripper']) @ inverse(delivery['T_world_gripper'])
                       if 'T_robot_gripper' in delivery and 'T_world_gripper' in delivery else None)
        if 'keypoints_world' in delivery:
            delivery['keypoints_world'] = {k: points(shift, v).tolist()
                                           for k, v in delivery['keypoints_world'].items()}
        for key in ('shoulder_midpoint', 'torso_center', 'hand_center_world'):
            if key in delivery: delivery[key] = points(shift, delivery[key]).tolist()
        for key in ('hand_direction_world', 'facing_direction_world'):
            if key in delivery: delivery[key] = (shift[:3,:3] @ delivery[key]).tolist()
        if 'T_world_object' in delivery:
            delivery['T_world_object'] = (shift @ transform(delivery['T_world_object'])).tolist()
            delivery['T_world_gripper'] = (transform(delivery['T_world_object']) @
                                          transform(trial['T_object_gripper'])).tolist()
            if robot_world is not None:
                delivery['T_robot_gripper'] = (robot_world @ transform(delivery['T_world_gripper'])).tolist()


def prepare(trial, robot):
    """Return the resolved trial and the calibrated Robotiq command.

    Resolved trials retain their receiver and grasp on subsequent runs. Changing
    assets requires reconversion from the original input rather than double fitting.
    """
    from .grasp_fit import fit_grasp
    result = deepcopy(trial)
    saved = result.get('asset_preparation')
    digest = mesh_digest(result['object_mesh_object'])
    if saved:
        if (saved.get('schema_version') != 'handover.asset_preparation.v1'
                or saved['robot'] != result['asset_robot'] or saved['mesh_sha256'] != digest):
            raise ValueError('Resolved replay assets changed; regenerate from the original trial')
        fit = result['asset_contact_fit']
        if fit['status'] == 'bilateral_surface_fit' and not np.allclose(
                result['T_object_gripper'], fit['T_object_gripper'], atol=1e-10, rtol=0):
            raise ValueError('Resolved grasp changed; regenerate from the original trial')
    else:
        original_grasp = transform(result['T_object_gripper'])
        oversized = projected_width(result['object_boxes'], original_grasp[:3,1]) > result['max_opening_m']
        selected = result.get('method_selection')
        if selected:
            prepared = selected.get('geometry_preparation') or {}
            if prepared.get('schema_version') != 'handover.asset_candidate.v1' or prepared.get('status') != 'bilateral_surface_fit':
                raise ValueError('Method grasp needs prepare-candidates with the original assets before selection; post-selection fitting is disabled')
            if prepared['robot'] != result['asset_robot'] or prepared['mesh_sha256'] != digest:
                raise ValueError('Selected candidate was prepared for different robot/object assets')
            if not np.allclose(original_grasp,prepared['T_object_gripper'],atol=1e-10,rtol=0):
                raise ValueError('Selected grasp differs from its pre-selection asset calibration')
            if not np.isclose(result['gripper_opening_m'],prepared['width_m'],atol=1e-7,rtol=0):
                raise ValueError('Selected opening differs from its asset contact width')
            fit = deepcopy(prepared)
            fit['contact_points_tool'] = deepcopy(prepared['contact_points_asset_tool'])
            result['T_asset_tool_grasp_frame'] = deepcopy(prepared['T_asset_tool_grasp_frame'])
        elif result.get('replay_reference', {}).get('assigned_outcome') == 'stability' and oversized:
            fit = {'status': 'intentional_width_failure',
                   'scope': 'No valid grasp; this reference failure is not corrected into a success'}
        else:
            fitted, fit = fit_grasp(result['object_mesh_object'], original_grasp, result['max_opening_m'])
            result['T_object_gripper'] = fitted.tolist()
    if fit['status'] == 'bilateral_surface_fit':
        opening, pads = robot.calibrate_opening(result['planned_joints'][-1], fit['width_m'])
        distances = robot.contact_distances(fit['contact_points_tool'])
        if not np.isfinite(distances).all() or len(distances) != 2 or max(distances) > .0002:
            raise ValueError(f"{result['object_id']}: missing bilateral USD pad contacts: {distances}")
        fit.update(measured_pad_geometry=pads, linkage_command_m=opening, bilateral_distance_m=distances,
                   contact_sides=['right', 'left'])
    elif fit['status'] == 'intentional_width_failure':
        if projected_width(result['object_boxes'], transform(result['T_object_gripper'])[:3,1]) <= result['max_opening_m']:
            raise ValueError('Resolved width-failure geometry changed; regenerate from the original trial')
        opening = .085
        robot.update(result['planned_joints'][-1], opening)
    else:
        raise ValueError('Unknown resolved contact status')
    old_goal = tcp(result['planned_joints'][-1])
    offset = inverse(old_goal) @ robot.tool_pose() @ transform(result.get('T_asset_tool_grasp_frame',np.eye(4)))
    if saved:
        if not np.allclose(offset, result['T_tcp_asset_tool'], atol=1e-5, rtol=0):
            raise ValueError('Resolved USD tool frame changed; regenerate from the original trial')
    else:
        old_object = old_goal @ inverse(original_grasp)
        new_object = old_goal @ offset @ inverse(result['T_object_gripper'])
        shift = new_object @ inverse(old_object)
        result['T_tcp_asset_tool'] = offset.tolist()
        fixed = result.get('receiver_protocol',{}).get('policy') == 'fixed_world'
        if fixed:
            shift = np.eye(4)
            result['target_T_world_gripper'] = (transform(result['target_T_world_object']) @
                                                transform(result['T_object_gripper'])).tolist()
        else:
            result['target_T_world_gripper'] = (transform(result['target_T_world_gripper']) @ offset).tolist()
        if 'delivery' in result: result['pre_asset_delivery'] = deepcopy(result['delivery'])
        if not fixed: move_receiver(result, shift)
        if 'planning' in result: result['pre_asset_planning'] = result.pop('planning')
        result['asset_preparation'] = {
            'schema_version': 'handover.asset_preparation.v1', 'robot': deepcopy(result['asset_robot']),
            'mesh_sha256': digest, 'T_world_receiver_retarget': shift.tolist(),
            'receiver_policy': 'fixed_world' if fixed else 'Rigidly retargeted with the object at the planned endpoint',
            'original_experiment': deepcopy(result.get('experiment')),
        }
        if 'experiment' in result:
            result['experiment']['receiver_policy'] = result['asset_preparation']['receiver_policy']
            result['experiment']['paired_receiver_preserved'] = fixed
    result['asset_contact_fit'] = fit
    if fit['status'] == 'bilateral_surface_fit':
        result.setdefault('gripper_opening_m',fit['width_m'])
        result.setdefault('gripper_opening_source','original_mesh_pad_contact')
    return result, opening
