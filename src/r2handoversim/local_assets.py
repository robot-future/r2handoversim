"""Explicit local USD/OBJ assets; no asset copies or machine paths in releases."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import numpy as np


def configuration(path):
    path=Path(path).resolve();data=json.loads(path.read_text())
    if not isinstance(data, dict) or data.get('schema_version')!='handover.local_assets.v1':
        raise ValueError('Expected handover.local_assets.v1')
    for key in ['robot_usd','object_mesh_root']:
        target=Path(data[key]);target=target if target.is_absolute() else path.parent/target
        target=target.resolve()
        if not target.exists(): raise ValueError(f'Missing local asset: {target}')
        data[key]=str(target)
    if not Path(data['robot_usd']).is_file(): raise ValueError('robot_usd must be a file')
    if not Path(data['object_mesh_root']).is_dir(): raise ValueError('object_mesh_root must be a directory')
    translation = np.asarray(data.get('robot_translation', [-.5625, -.36, -.02]), dtype=float)
    if translation.shape != (3,) or not np.isfinite(translation).all():
        raise ValueError('robot_translation must contain three finite meter coordinates')
    aliases = data.get('object_aliases', {})
    if not isinstance(aliases, dict) or not all(isinstance(k, str) and isinstance(v, str) for k,v in aliases.items()):
        raise ValueError('object_aliases must map string object IDs to string mesh names')
    return data


def attach(trials, config):
    import trimesh
    result=[];cache={}
    for trial in trials:
        t=deepcopy(trial);name=config.get('object_aliases',{}).get(t['object_id'],t['object_id'])
        root=Path(config['object_mesh_root']);path=(root/(name+'.obj')).resolve()
        if not path.is_relative_to(root) or not path.is_file(): raise ValueError(f'Object mesh unavailable: {name}')
        if name not in cache:
            mesh=trimesh.load(path,force='mesh',process=False)
            if not len(mesh.faces): raise ValueError(f'Object has no triangle faces: {name}')
            colors=mesh.visual.to_color().vertex_colors if mesh.visual.kind=='texture' else mesh.visual.vertex_colors
            cache[name]={'vertices':np.asarray(mesh.vertices).tolist(),'faces':np.asarray(mesh.faces).tolist(),
                         'colors':(np.asarray(colors)[:,:3]/255.).tolist(), 'source_path':str(path),
                         'source_sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
        t['object_mesh_object']=cache[name]
        t['asset_robot']={'usd':config['robot_usd'],'translation':config.get('robot_translation',[-.5625,-.36,-.02])}
        result.append(t)
    return result
