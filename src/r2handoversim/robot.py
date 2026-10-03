"""UR5e nominal DH kinematics and explicit box collision proxies.

Provides forward kinematics and proxy shapes for joint-trajectory replay.
DH dimensions follow Universal Robots' published nominal parameters.
"""
import numpy as np
from .geometry import box, pose, unit, vector

A = (0., -.425, -.3922, 0., 0., 0.)
D = (.1625, 0., 0., .1333, .0997, .0996)
ALPHA = (np.pi / 2, 0., 0., np.pi / 2, -np.pi / 2, 0.)
BASE = pose([0, 0, .75])
HOME = np.array([0., -1.57, 1.57, -1.57, -1.57, 0.])
GOAL = np.array([.25, -1.10, 1.45, -1.90, -1.57, .25])


def joints(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError("Expected six finite UR5e joint angles in radians")
    return q


def frames(q):
    t, result = BASE.copy(), [BASE.copy()]
    for theta, a, d, alpha in zip(joints(q), A, D, ALPHA):
        c, s, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
        t = t @ np.array([[c, -s*ca, s*sa, a*c], [s, c*ca, -c*sa, a*s],
                          [0, sa, ca, d], [0, 0, 0, 1.]])
        result.append(t.copy())
    return result


def tcp(q):
    return frames(q)[-1] @ pose([0, 0, .12])


def segment_box(start, end, radius=.028, label="arm"):
    start, end = vector(start), vector(end)
    delta = end - start
    length = float(np.linalg.norm(delta))
    z = unit(delta) if length > 1e-9 else np.array([0., 0., 1.])
    helper = np.array([1., 0., 0.]) if abs(z[0]) < .8 else np.array([0., 1., 0.])
    x = unit(np.cross(helper, z))
    y = np.cross(z, x)
    return box((start + end) / 2, [radius, radius, max(length / 2, radius)],
               np.column_stack([x, y, z]), label)


def arm_boxes(q):
    transforms = frames(q) + [tcp(q)]
    return [segment_box(a[:3, 3], b[:3, 3], label=f"link_{i}")
            for i, (a, b) in enumerate(zip(transforms[:-1], transforms[1:]))]


def interpolate(start, end, steps=90):
    if steps < 2:
        raise ValueError("At least two trajectory samples required")
    u = np.linspace(0, 1, steps)
    u = 3*u*u - 2*u*u*u
    return (joints(start) + u[:, None] * (joints(end) - joints(start))).tolist()


def solve_position(target, initial=GOAL, iterations=180):
    """Damped least-squares position IK for constructing illustrative deviations."""
    q = joints(initial).copy()
    target = vector(target)
    for _ in range(iterations):
        p = tcp(q)[:3, 3]
        error = target - p
        if np.linalg.norm(error) < .0005:
            return q
        jac = np.column_stack([(tcp(q + np.eye(6)[i]*1e-5)[:3, 3] - p)/1e-5 for i in range(6)])
        delta = jac.T @ np.linalg.solve(jac @ jac.T + .002*np.eye(3), error)
        q += np.clip(delta, -.08, .08)
    raise ValueError("Demo position IK did not converge")
