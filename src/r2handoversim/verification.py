"""Cross-check exported observations without re-running Isaac Sim."""
import json
from pathlib import Path
import numpy as np
from .evaluation import evaluate, trial_tcp
from .geometry import inverse, transform


def verify_output(directory):
    root = Path(directory).resolve()
    state = json.loads((root/'run.json').read_text())
    rows = json.loads((root/'results.json').read_text())
    if (state.get('status') != 'succeeded' or not rows
            or state.get('expected_trials') != len(rows) or state.get('completed_trials') != len(rows)):
        raise ValueError('Run is incomplete or its expected/completed trial count differs from results')
    if len({r['trial_id'] for r in rows}) != len(rows):
        raise ValueError('Duplicate trial IDs in exported results')
    frames = 0
    for row in rows:
        paths = {}
        for key, name in row['artifacts'].items():
            path = (root/name).resolve()
            if not path.is_relative_to(root) or not path.is_file() or path.stat().st_size == 0:
                raise ValueError(f'Missing, empty or external artifact: {name}')
            paths[key] = path
        trial = json.loads(paths['trial'].read_text())
        if trial['id'] != row['trial_id']:
            raise ValueError('Resolved trial ID differs from result ID')
        def equal(name, actual, expected, tolerance=1e-10):
            a, b = np.asarray(actual), np.asarray(expected)
            if a.shape != b.shape or not np.isfinite(a).all() or not np.allclose(a,b,atol=tolerance,rtol=0):
                raise ValueError(f"{trial['id']}: inconsistent {name}")
        with np.load(paths['trajectory'], allow_pickle=False) as data:
            if str(data['schema_version']) != 'handover.trajectory.v1':
                raise ValueError('Unsupported trajectory schema')
            q = np.asarray(trial['executed_joints'])
            contacts = data['robot_hand_contact']
            if contacts.dtype != np.bool_ or contacts.shape != (len(q),):
                raise ValueError('Contact observations must be one boolean per frame')
            expected = evaluate(trial, contacts)
            for field in ('metrics','success','first_failure','contact_frames','object_id','variant','split'):
                if row[field] != expected[field]:
                    raise ValueError(f"{trial['id']}: result {field} differs from resolved scene and observations")
            equal('duration', row['trajectory_duration_s'], (len(q)-1)*trial['dt_s'])
            equal('timestamps', data['time_s'], np.arange(len(q))*trial['dt_s'])
            equal('joints', data['joint_position_rad'], q)
            equal('grasp', data['T_object_gripper'], trial['T_object_gripper'])
            commanded = np.asarray([trial_tcp(trial,joint) for joint in q])
            equal('tool poses',data['T_world_tool'],commanded,1e-4)
            for pose in data['T_world_tool']: transform(pose)
            equal('object poses',data['T_world_object'],commanded@inverse(trial['T_object_gripper']))
            if 'receiver' in trial:
                receiver=trial['receiver']
                for key in ('id','side','seed'):
                    if str(data['receiver_'+key]) != str(receiver[key]):
                        raise ValueError('Receiver metadata differs from trajectory')
                equal('static receiver',data['T_world_hand'],np.repeat(np.asarray(receiver['T_world_hand'])[None],len(q),axis=0))
                equal('static palm',data['palm_position_world'],np.repeat(np.asarray(trial['palm_position_world'])[None],len(q),axis=0))
            frames += len(q)
    return {'schema_version':'handover.output_verification.v1', 'status':'passed',
            'trials':len(rows), 'frames':frames,
            'scope':'Internal consistency check of saved replay exports'}
