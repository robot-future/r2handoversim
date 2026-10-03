"""Choose a review camera that keeps receiver vertices clear of asset bounds."""
import numpy as np


def visibility(eye, targets, bounds):
    targets=np.asarray(targets,dtype=float);bounds=np.asarray(bounds,dtype=float)
    if not len(bounds): return 1.
    direction=targets-np.asarray(eye)
    parallel=np.abs(direction)<1e-12
    denominator=np.where(parallel,1.,direction)
    a=(bounds[None,:,0,:]-eye)/denominator[:,None,:]
    b=(bounds[None,:,1,:]-eye)/denominator[:,None,:]
    lo=np.minimum(a,b);hi=np.maximum(a,b)
    inside=(eye>=bounds[:,0,:]) & (eye<=bounds[:,1,:])
    lo=np.where(parallel[:,None,:],np.where(inside[None],-np.inf,np.inf),lo)
    hi=np.where(parallel[:,None,:],np.where(inside[None],np.inf,-np.inf),hi)
    near=lo.max(2);far=hi.min(2)
    blocked=(far>=np.maximum(near,0.)) & (near<.995)
    return float(np.mean(~blocked.any(1)))


def choose(hand_vertices,target,bounds_by_pose,scale=1.,view="handover",object_vertices=None):
    hand=np.asarray(hand_vertices)[::max(1,len(hand_vertices)//64)]
    target=np.asarray(target)
    base_angle=np.arctan2(target[1],target[0])  # receiver side, looking toward robot
    offsets=[0.,np.pi/6,-np.pi/6,np.pi/3,-np.pi/3,np.pi/2,-np.pi/2,2*np.pi/3,-2*np.pi/3,np.pi]
    heights=(.7,1.1)
    if view in ("left","right"):
        base_angle += (1 if view=="left" else -1)*np.pi/3
        offsets=[0.,np.pi/12,-np.pi/12,np.pi/6,-np.pi/6]
    elif view=="top":
        heights=(1.7,)
    elif view!="handover":
        raise ValueError("Unknown detail camera")
    best=None
    for height in heights:
        for offset in offsets:
            angle=base_angle+offset
            eye=target+scale*np.array([np.cos(angle),np.sin(angle),height])
            visible=[visibility(eye,hand,bounds) for bounds in bounds_by_pose]
            object_visible=None
            if object_vertices is not None:
                samples=np.asarray(object_vertices)[::max(1,len(object_vertices)//64)]
                # Caller appends the held object's own bound last; don't count self-occlusion.
                object_visible=visibility(eye,samples,bounds_by_pose[-1][:-1])
            score=min(visible)+.1*np.mean(visible)+(.3*object_visible if object_visible is not None else 0.)
            if best is None or score>best[0]+1e-9: best=(score,eye,visible,object_visible)
    return best[1],{'sampled_pose_visibility':best[2], 'terminal_object_visibility':best[3],
                   'visibility_rule':'Hand vertex rays against original asset world AABBs; camera heuristic only'}
