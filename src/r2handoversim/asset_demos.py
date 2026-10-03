"""Standalone mesh demos, without method weights or dataset manifests.

Approaches, receiving hands and trajectories are authored geometric fixtures.
"""
import json
from pathlib import Path
import re
import numpy as np
from .local_assets import configuration, attach
from .demos import load_demo
from .geometry import box, inverse, moved, points
from .grasp_fit import fit_grasp
from .robot import GOAL, HOME, tcp, interpolate
from .evaluation import validate_trial


def approach(mesh):
    """Choose a deterministic face approach with a feasible local pad span."""
    vertices = np.asarray(mesh['vertices'])
    lo, hi = vertices.min(0), vertices.max(0)
    # Prefer entering along the longest dimension, closing across a short one.
    for z_axis in np.argsort(-(hi-lo)):
        for sign in (-1, 1):
            z = np.eye(3)[z_axis]*sign
            for y_axis in sorted(set(range(3))-{int(z_axis)}, key=lambda k: hi[k]-lo[k]):
                y = np.eye(3)[y_axis]; x = np.cross(y, z)
                rotation = np.column_stack([x,y,z])
                local = vertices @ rotation
                lower, upper = local.min(0), local.max(0)
                for x_fraction in (.5, .25, .75):
                    grasp = np.eye(4); grasp[:3,:3] = rotation
                    origin = (lower+upper)/2
                    origin[0] = lower[0] + x_fraction*(upper[0]-lower[0])
                    origin[2] = lower[2]
                    grasp[:3,3] = rotation@origin
                    try:
                        fit_grasp(mesh, grasp)
                        return grasp
                    except ValueError:
                        pass
    raise ValueError('No local mesh approach fits the 85 mm pad aperture; provide an explicit trial')


def generate(config_path, output, names=None):
    from .planning import solve_pose
    config = configuration(config_path)
    names = names or sorted(p.stem for p in Path(config['object_mesh_root']).glob('*.obj'))
    if not names or len(set(names)) != len(names):
        raise ValueError('Choose at least one distinct object ID')
    if any(not re.fullmatch(r'[a-zA-Z0-9_-]+', n) for n in names):
        raise ValueError('Object IDs must contain letters, digits, underscores or hyphens')
    target = tcp(GOAL); target[2,3] += .23
    goals = solve_pose(target, initial=GOAL)
    if not goals: raise ValueError('Unable to solve the demonstration endpoint')
    path = interpolate(HOME, goals[0], 90)
    trials = []
    for name in names:
        trial = load_demo('bottle'); trial['object_id'] = name
        trial = attach([trial], config)[0]
        mesh = trial['object_mesh_object']
        grasp = approach(mesh)
        verts = np.asarray(mesh['vertices']); lo, hi = verts.min(0), verts.max(0)
        # Use explicitly coarse boxes for metric queries; render the original mesh.
        trial['object_boxes'] = [box((lo+hi)/2, np.maximum((hi-lo)/2, .001), label='mesh_bounds_proxy')]
        trial['usage_boxes'] = []
        local = (verts-grasp[:3,3])@grasp[:3,:3]
        palm_tool = (local.min(0)+local.max(0))/2
        palm_tool[2] = local[:,2].max()+.08
        palm_object = points(grasp, palm_tool)
        world_object = target@inverse(grasp)
        hand_object = box(palm_object, [.025,.035,.015], rotation=grasp[:3,:3], label='authored_hand_proxy')
        trial.update(id=f'{name}_local_mesh', variant='local_mesh_demo', split='S0',
            T_object_gripper=grasp.tolist(), target_T_world_gripper=target.tolist(),
            hand_boxes_world=[moved(hand_object, world_object)],
            palm_position_world=points(world_object,palm_object).tolist(),
            palm_normal_world=(-target[:3,2]).tolist(),
            planned_joints=path, executed_joints=path,
            provenance='Local mesh; authored geometric grasp approach, hand proxy and joint replay',
            annotation_status='Geometry-only S0 demonstration with generated region annotations',
            source_data={'mesh_sha256':mesh['source_sha256'], 'mesh_path':mesh['source_path']})
        validate_trial(trial)
        trials.append(trial)
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    (output/'trials.json').write_text(json.dumps(trials, allow_nan=False))
    (output/'conversion.json').write_text(json.dumps({'schema_version':'handover.asset_demos.v1',
        'objects':names, 'trials':len(trials), 'source_config':str(Path(config_path).resolve()),
        'scope':'Generated geometry demos; original local meshes and configured UR5e/Robotiq'}, indent=2))
    return trials
