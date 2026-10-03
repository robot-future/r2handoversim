"""Save the resolved scene and frame-aligned replay observations."""
import json
from pathlib import Path
import numpy as np
from .evaluation import trial_tcp, validate_trial
from .geometry import inverse, transform


def export(trial, contacts, output, observed_tool_poses=None):
    validate_trial(trial)
    q = np.asarray(trial['executed_joints'], dtype=float)
    contacts = np.asarray(contacts)
    if contacts.shape != (len(q),) or contacts.dtype != np.bool_:
        raise ValueError('Trajectory export requires one boolean contact observation per frame')
    commanded = np.asarray([trial_tcp(trial, row) for row in q])
    observed = commanded if observed_tool_poses is None else np.asarray(observed_tool_poses, dtype=float)
    if observed.shape != commanded.shape:
        raise ValueError('Observed tool poses must cover every replay frame')
    for pose in observed: transform(pose)
    if not np.allclose(observed, commanded, atol=1e-4, rtol=0):
        raise ValueError('USD tool motion differs from the replay kinematic model')
    objects = commanded @ inverse(trial['T_object_gripper'])
    output = Path(output)
    scene = output / f"{trial['id']}_trial.json"
    trajectory = output / f"{trial['id']}_trajectory.npz"
    scene.write_text(json.dumps(trial, allow_nan=False))
    receiver = trial.get('receiver')
    extra = {} if receiver is None else dict(
        receiver_id=receiver['id'], receiver_side=receiver['side'], receiver_seed=receiver['seed'],
        T_world_hand=np.repeat(np.asarray(receiver['T_world_hand'])[None],len(q),axis=0),
        palm_position_world=np.repeat(np.asarray(trial['palm_position_world'])[None],len(q),axis=0))
    np.savez_compressed(trajectory, **extra, schema_version='handover.trajectory.v1',
        pose_source='USD tool observations' if observed_tool_poses is not None else 'nominal kinematics',
        time_s=np.arange(len(q))*trial['dt_s'], joint_position_rad=q,
        T_world_tool=observed, T_world_object=objects,
        T_object_gripper=np.asarray(trial['T_object_gripper']), robot_hand_contact=contacts,
        max_tool_model_error_m=np.max(np.linalg.norm(observed[:,:3,3]-commanded[:,:3,3], axis=1)))
    return {'trial': scene.name, 'trajectory': trajectory.name}
