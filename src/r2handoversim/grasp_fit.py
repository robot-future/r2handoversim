"""Fit a replay grasp to opposing mesh surfaces inside the actual pad window.

Computes geometric contact placement for kinematic replay.
"""
import numpy as np
from .geometry import transform


def clipped_surface(vertices, faces, bounds):
    """Vertices of triangle surfaces clipped to an X/Z rectangular prism."""
    tri=vertices[faces]
    active=np.ones(len(tri),dtype=bool)
    for axis,lo,hi in bounds:
        active &= (tri[:,:,axis].max(1)>=lo)&(tri[:,:,axis].min(1)<=hi)
    tri=tri[active]
    if not len(tri): return np.empty((0,3))
    pts=tri.reshape(-1,3); candidates=[pts]
    a=tri.reshape(-1,3);b=np.roll(tri,-1,axis=1).reshape(-1,3)
    for axis,lo,hi in bounds:
        for edge in (lo,hi):
            delta=b[:,axis]-a[:,axis]
            mask=np.abs(delta)>1e-12
            u=(edge-a[mask,axis])/delta[mask]
            valid=(u>=0)&(u<=1)
            candidates.append(a[mask][valid]+u[valid,None]*(b-a)[mask][valid])
    # A large triangle can cover a rectangle corner without an interior vertex.
    v0=tri[:,0];v1=tri[:,1]-v0;v2=tri[:,2]-v0
    det=v1[:,0]*v2[:,2]-v1[:,2]*v2[:,0]
    mask=np.abs(det)>1e-12
    for x in bounds[0][1:]:
        for z in bounds[1][1:]:
            dx=x-v0[mask,0];dz=z-v0[mask,2]
            u=(dx*v2[mask,2]-dz*v2[mask,0])/det[mask]
            v=(v1[mask,0]*dz-v1[mask,2]*dx)/det[mask]
            valid=(u>=0)&(v>=0)&(u+v<=1)
            candidates.append(v0[mask][valid]+u[valid,None]*v1[mask][valid]+v[valid,None]*v2[mask][valid])
    pts=np.concatenate(candidates)
    keep=np.ones(len(pts),dtype=bool)
    for axis,lo,hi in bounds: keep &= (pts[:,axis]>=lo-1e-10)&(pts[:,axis]<=hi+1e-10)
    return pts[keep]


def fit_grasp(mesh, grasp, max_opening=.085):
    original=transform(grasp);verts=np.asarray(mesh['vertices']);faces=np.asarray(mesh['faces'],dtype=int)
    local=(verts-original[:3,3])@original[:3,:3]
    # Stay near the supplied approach, preferring a shallow insertion. The pad
    # window is inset 1 mm from the original Robotiq USD's flat inner faces.
    xs=np.unique(np.r_[0.,np.linspace(local[:,0].min()+.008,local[:,0].max()-.008,11)])
    options=[]
    for x in xs:
        for depth in (.018,.025,.035):
            options.append((float(abs(x)+depth*.2),float(x),depth))
    for _,x,depth in sorted(options):
        pts=clipped_surface(local,faces,[(0,x-.010,x+.010),(2,depth-.022,depth+.010)])
        if len(pts)<4: continue
        low=pts[np.argmin(pts[:,1])];high=pts[np.argmax(pts[:,1])]
        width=float(high[1]-low[1])
        if not .001<=width<=max_opening-.0001: continue
        # Require contacts within the flat pads, with a nontrivial insertion.
        if max(low[2],high[2])<.002: continue
        center=np.array([x,(low[1]+high[1])/2,depth])
        corrected=original.copy();corrected[:3,3]+=original[:3,:3]@center
        contacts=np.array([low,high])-center
        return corrected, {'status':'bilateral_surface_fit','width_m':width,
            'original_T_object_gripper':original.tolist(),'T_object_gripper':corrected.tolist(),
            'translation_in_original_tool_m':center.tolist(),'contact_points_tool':contacts.tolist(),
            'pad_window_tool_m':{'x':[-.010,.010],'z':[-.022,.010]},
            'scope':'Opposing mesh surfaces inside flat finger pads; kinematic contact placement'}
    raise ValueError('No opposing object surfaces fit within the Robotiq pad window and aperture')
