"""Seeded static receiver poses from explicitly configured local hand meshes."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import numpy as np
from .geometry import box, inverse, moved, points, pose, transform, unit
from .robot import HOME


def read_templates(path):
    import trimesh
    path = Path(path).resolve()
    config = json.loads(path.read_text())
    if config.get('schema_version') != 'handover.receivers.v1':
        raise ValueError('Expected handover.receivers.v1')
    templates = []
    for item in config['templates']:
        mesh_path = (path.parent/item['mesh']).resolve()
        scale = float(item['scale_to_m'])
        if not np.isfinite(scale) or scale <= 0: raise ValueError('Receiver scale must be positive')
        if item['side'] not in ('left', 'right'): raise ValueError('Receiver side must be left or right')
        mesh = trimesh.load(mesh_path, force='mesh', process=False)
        vertices = np.asarray(mesh.vertices)*scale
        if not len(mesh.faces) or not np.isfinite(vertices).all(): raise ValueError('Invalid receiver mesh')
        palm = np.asarray(item['palm_position_mesh'], dtype=float)*scale
        z = unit(item['palm_normal_mesh'])
        finger = unit(item['finger_direction_mesh'])
        x = unit(finger-z*(finger@z)); y = np.cross(z,x)
        frame = pose(palm, np.column_stack([x,y,z]))
        # The canonical palm frame has +Z outward, +X toward the fingertips.
        local = points(inverse(frame), vertices)
        if not .06 < np.ptp(local,axis=0).max() < .35:
            raise ValueError('Hand scale is implausible; supply explicit scale_to_m')
        templates.append(dict(id=item['id'], side=item['side'], vertices=local.tolist(),
            faces=np.asarray(mesh.faces,dtype=int).tolist(),
            source={'path':str(mesh_path),'sha256':hashlib.sha256(mesh_path.read_bytes()).hexdigest(),
                    'scale_to_m':scale,'palm_frame_in_mesh_m':frame.tolist(),
                    'kind':item.get('source_kind','locally supplied hand mesh')}))
    if not templates or len({t['id'] for t in templates}) != len(templates):
        raise ValueError('Receiver templates must have distinct IDs')
    return config, templates


def sample_pose(rng, sampling):
    from scipy.spatial.transform import Rotation
    low, high = np.asarray(sampling['position_bounds_m'], dtype=float)
    angles = np.asarray(sampling['rpy_bounds_deg'], dtype=float)
    if low.shape != (3,) or high.shape != (3,) or angles.shape != (2,3):
        raise ValueError('Receiver sampling bounds must each have shape (2,3)')
    if not np.isfinite([low,high,*angles]).all() or np.any(low>=high) or np.any(angles[0]>angles[1]):
        raise ValueError('Invalid receiver sampling bounds')
    return pose(rng.uniform(low,high), Rotation.from_euler('xyz',rng.uniform(*angles),degrees=True).as_matrix())


def apply_receiver(trial, template, hand_pose, seed, index):
    """Receiver pose is sampled independently of the selected grasp and never retargeted."""
    result = deepcopy(trial)
    hand_pose = transform(hand_pose)
    local = np.asarray(template['vertices'])
    world = points(hand_pose,local)
    # Spatial cells avoid one large palm AABB filling all gaps between fingers.
    low, high = local.min(0),local.max(0)
    cells = np.minimum(((local-low)/np.maximum(high-low,1e-6)*4).astype(int),3)
    boxes = []
    for cell in np.unique(cells,axis=0):
        v = local[np.all(cells==cell,axis=1)]
        center=(v.min(0)+v.max(0))/2; ext=np.maximum((v.max(0)-v.min(0))/2,.002)
        boxes.append(moved(box(center,ext,label='receiver_mesh_cell'),hand_pose))
    result['hand_mesh_world']={'vertices':world.tolist(),'faces':deepcopy(template['faces'])}
    result['hand_boxes_world']=boxes
    result['palm_position_world']=hand_pose[:3,3].tolist()
    result['palm_normal_world']=hand_pose[:3,2].tolist()
    result['receiver']={'id':f"receiver_{seed}_{index:04d}_{template['side']}", 'template_id':template['id'],
        'side':template['side'], 'seed':seed, 'sample_index':index,'T_world_hand':hand_pose.tolist(),
        'static_world':True,'source':deepcopy(template['source']),
        'sampling_scope':'Seeded SE(3) placement of supplied local hand templates'}
    result['receiver_protocol']={'policy':'fixed_world','replan_in_isaac':True,
        'hand_collision':'mesh','object_collision':'convexHull', 'planner':'isaacsim_physx_rrt_connect'}
    return result


def object_target(hand_pose, reference):
    g=transform(reference['T_object_gripper'])
    z=-hand_pose[:3,2];x=hand_pose[:3,0];y=np.cross(z,x)
    rotation=np.column_stack([x,y,z])@g[:3,:3].T
    vertices=np.asarray(reference['object_mesh_object']['vertices'])
    center=(vertices.min(0)+vertices.max(0))/2
    return pose(hand_pose[:3,3]+.15*hand_pose[:3,2]-rotation@center,rotation)


def generate(trials, config_path, output, samples=4, seed=0):
    from scipy.spatial.transform import Rotation
    from .evaluation import validate_trial
    if any(t.get('method_selection') and t['method_selection'].get('mode')!='receiver_setup' for t in trials):
        raise ValueError('Sample receiver-scenes BEFORE method selection; selected method trials cannot be resampled')
    config, templates = read_templates(config_path)
    if samples < 1: raise ValueError('Receiver sample count must be positive')
    references={}
    for trial in trials:
        name=trial['object_id']
        if name not in references or trial.get('method_selection',{}).get('mode')=='FS':
            references[name]=trial
    # Define reachability using a reference pose BEFORE selecting any method.
    # Rejected proposals are logged; collision failures are never resampled.
    banks={}; proposals={}
    require_ik=bool(config['sampling'].get('require_reference_ik',False))
    for name in sorted(references):
        reference=references[name]
        entropy=int.from_bytes(hashlib.sha256(name.encode()).digest()[:4],'little')
        rng=np.random.default_rng(np.random.SeedSequence([seed,entropy]))
        banks[name]=[];proposals[name]=[]
        for i in range(samples):
            template=templates[i%len(templates)]
            for attempt in range(int(config['sampling'].get('max_proposals_per_sample',100))):
                hand_pose=sample_pose(rng,config['sampling'])
                accepted=True
                if require_ik:
                    from .planning import solve_pose
                    offset=reference.get('T_tcp_asset_tool') or reference.get('method_selection',{}).get('geometry_preparation',{}).get('T_tcp_asset_tool')
                    if offset is None: raise ValueError('Reachable-set sampling requires a calibrated T_tcp_asset_tool reference')
                    target=object_target(hand_pose,reference)@transform(reference['T_object_gripper'])@inverse(offset)
                    accepted=bool(solve_pose(target,initial=HOME,seed=seed+i))
                proposals[name].append({'sample_index':i,'attempt':attempt,'T_world_hand':hand_pose.tolist(),
                    'reference_ik_feasible':accepted if require_ik else None,'accepted':accepted})
                if accepted:
                    banks[name].append((template,hand_pose));break
            else: raise ValueError(f'No reference-IK reachable receiver for {name} sample {i}; widen or correct explicit bounds')
    result=[]
    for trial in trials:
        if 'object_mesh_object' not in trial or 'asset_robot' not in trial:
            raise ValueError('Random receiver protocol requires original robot/object assets')
        for index,(template,hand_pose) in enumerate(banks[trial['object_id']]):
            t=apply_receiver(trial,template,hand_pose,seed,index)
            t['receiver']['id']=f"{trial['object_id']}_{t['receiver']['id']}"
            t['id']=f"{trial['id']}_receiver_{index:04d}"
            # Author a common object target in the palm-normal Reach region.
            # This is an offline target demo, not a learned baseline prediction.
            g=transform(t['T_object_gripper'])
            object_pose=object_target(hand_pose,references[t['object_id']])
            t['target_T_world_object']=object_pose.tolist()
            t['target_T_world_gripper']=(object_pose@g).tolist()
            t['planned_joints']=[HOME.tolist()];t['executed_joints']=[HOME.tolist()]
            for key in ('planning','asset_preparation','asset_contact_fit','T_tcp_asset_tool'):
                t.pop(key,None)
            t['receiver_protocol']['sampling_bounds']=deepcopy(config['sampling'])
            t['receiver_protocol']['target_rule']='Object bounding center 15 cm along outward palm normal; authored offline target'
            t['receiver_protocol']['object_target_policy']='Fixed across input variants for this object/receiver; shared preselection reference orientation'
            t['receiver_protocol']['planning_seed']=seed+index
            validate_trial(t);result.append(t)
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    (output/'trials.json').write_text(json.dumps(result,allow_nan=False))
    (output/'receivers.json').write_text(json.dumps({'schema_version':'handover.receiver_bank.v1','seed':seed,
        'samples':samples,'templates':len(templates),'sampling':config['sampling'],
        'proposal_log':proposals,'reachable_set':'reference full-pose IK conditioned' if require_ik else 'bounded workspace only; no IK conditioning',
        'receivers':list({t['receiver']['id']:t['receiver'] for t in result}.values())},indent=2))
    return result


def scene_batch(scene_path, config_path, output, samples=4, seed=0):
    """Expose each sampled receiver to the method BEFORE candidate selection."""
    from .demos import from_selection
    scene=json.loads(Path(scene_path).read_text())
    contract=scene['gripper'].get('geometry_contract')
    if not contract: raise ValueError('Run prepare-candidates before receiver-scenes')
    reference=next((c for c in scene['candidates'] if c.get('geometry_preparation',{}).get('status')=='bilateral_surface_fit'),None)
    if reference is None: raise ValueError('No prepared reference grasp for the common object target')
    prepared=reference['geometry_preparation']
    selected={**deepcopy(reference),'width_m':prepared['width_m'],'width_source':'asset_mesh_pad'}
    selection={'schema_version':'handover.selection.v1','status':'ok','object_id':scene['object']['id'],
               'selected':selected,'mode':'receiver_setup','grasp_contract':{
                   'frame':'parallel_jaw_tip','closing_axis':'+Y','approach_axis':'+Z',
                   'width_policy':'asset_mesh_pad','max_opening_m':scene['gripper']['max_opening_m']}}
    trial=from_selection(scene,selection,variant='receiver_setup')
    trial['asset_robot']=deepcopy(contract['robot'])
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    trials=generate([trial],config_path,output/'setup',samples,seed)
    entries=[]
    for trial in trials:
        updated=deepcopy(scene);object_world=transform(trial['target_T_world_object']);inv=inverse(object_world)
        hand=trial['hand_mesh_world'];hand_pose=transform(trial['receiver']['T_world_hand'])
        updated['receiving_hand']={
            'center':points(inv,trial['palm_position_world']).tolist(),
            'direction':(inv[:3,:3]@hand_pose[:3,0]).tolist(),
            'palm_normal':(inv[:3,:3]@trial['palm_normal_world']).tolist(),
            'boxes':[moved(b,inv) for b in trial['hand_boxes_world']],
            'mesh':{'vertices':points(inv,hand['vertices']).tolist(),'faces':deepcopy(hand['faces'])},
            'provenance':'Sampled static MANO receiver expressed in the shared object target frame'}
        updated['receiver']=deepcopy(trial['receiver'])
        updated['receiver_protocol']=deepcopy(trial['receiver_protocol'])
        updated['target_T_world_object']=trial['target_T_world_object']
        updated['intent']['hand']=trial['receiver']['side']
        name=f"{trial['receiver']['id']}_scene.json"
        (output/name).write_text(json.dumps(updated,allow_nan=False))
        entries.append({'object_id':trial['object_id'],'receiver_id':trial['receiver']['id'],'scene':name})
    manifest={'schema_version':'handover.receiver_scenes.v1','seed':seed,'scenes':entries,
              'scope':'Methods must select separately on each scene; object and receiver targets are fixed across modes'}
    (output/'scenes.json').write_text(json.dumps(manifest,indent=2))
    return manifest
