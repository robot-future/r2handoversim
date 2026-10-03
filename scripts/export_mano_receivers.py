#!/usr/bin/env python3
"""Export neutral MANO rest meshes from the user's locally trusted model files.

No model assets are distributed. This uses the zero-shape template directly;
it does not decode a motion sequence, predict hand poses or require Torch.
"""
import argparse
import inspect
import json
import pickle
from collections import namedtuple
from pathlib import Path
import numpy as np


def read_model(path):
    # Original MANO files contain legacy Chumpy objects. Compatibility aliases
    # exist only during loading; this must only read trusted local model files.
    aliases={'bool':bool,'int':int,'float':float,'complex':complex,'object':object,'unicode':str,'str':str}
    added=[];patched=not hasattr(inspect,'getargspec')
    try:
        for name,value in aliases.items():
            if name not in np.__dict__: setattr(np,name,value);added.append(name)
        if patched:
            spec=namedtuple('ArgSpec','args varargs keywords defaults')
            def getargspec(function):
                full=inspect.getfullargspec(function)
                return spec(full.args,full.varargs,full.varkw,full.defaults)
            inspect.getargspec=getargspec
        with Path(path).open('rb') as stream: data=pickle.load(stream,encoding='latin1')
        return {key:np.asarray(value.r) if hasattr(value,'r') else value for key,value in data.items()}
    finally:
        for name in added: delattr(np,name)
        if patched: delattr(inspect,'getargspec')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--models',type=Path,required=True,help='Directory containing trusted MANO_LEFT.pkl and MANO_RIGHT.pkl')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    templates=[]
    for side in ('left','right'):
        data=read_model(args.models/f'MANO_{side.upper()}.pkl')
        vertices=np.asarray(data['v_template']);faces=np.asarray(data['f'],dtype=int)
        joints=np.asarray(data['J_regressor']@vertices)
        if vertices.shape!=(778,3) or not np.isfinite(vertices).all(): raise ValueError('Expected finite MANO template vertices')
        normal=np.cross(joints[1]-joints[0],joints[7]-joints[0]);normal/=np.linalg.norm(normal)
        if side=='left': normal=-normal
        path=args.output/f'mano_{side}.obj'
        path.write_text(''.join('v '+' '.join(map(str,v))+'\n' for v in vertices)
            +''.join('f '+' '.join(map(str,f+1))+'\n' for f in faces))
        templates.append(dict(id=f'mano_open_{side}',side=side,mesh=path.name,scale_to_m=1.,
            palm_position_mesh=joints[[0,1,4,7,10]].mean(0).tolist(),palm_normal_mesh=normal.tolist(),
            finger_direction_mesh=(joints[4]-joints[0]).tolist(),
            source_kind='Neutral MANO rest template from locally licensed model; not an interaction sequence'))
    config=dict(schema_version='handover.receivers.v1',templates=templates,sampling={
        'position_bounds_m':[[-.72,-.45,.88],[-.60,-.30,.98]],
        'rpy_bounds_deg':[[-25,-25,-150],[25,25,-50]],'require_reference_ik':True})
    (args.output/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    print((args.output/'config.json').resolve())


if __name__=='__main__': main()
