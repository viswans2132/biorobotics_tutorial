import math
import numpy as np

body_x, body_y, body_z = 0.4, 0.16, 0.1
hip_len = 0.06
thigh_len = 0.2
shank_len = 0.2

hip_rad = 0.018
hip_clear_y = hip_rad + 0.02
hip_drop_z = hip_rad + 0.01

knee_clear_y = 0.05
knee_fwd_x = 0.02

JOINT_LIMITS = {
    'abd': (-0.8, 0.8),
    'hip': (-1.5, 1.5),
    'knee': (-2.7, 0.1)
}


def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([1, 0, 0],
                    [0, c, -s],
                    [0, s, c])


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def in_limits(q, lim):
    return (q >= lim[0] - 1e-9) and (q <= lim[1] + 1e-9)


def leg_mount_in_trunk (xsign, ysign):
    x_mount = xsign * (body_x / 2)
    y_mount = ysign * (body_y / 2 + hip_clear_y)
    z_mount = -(body_z / 2 + hip_drop_z)
    return np.array([x_mount, y_mount, z_mount])


def ik_leg_3dof(p_trunk, xsign, ysign, return_all=False):
    p = np.asarray(p_trunk, dtype=float).reshape(3)
    r0 = leg_mount_in_trunk(xsign, ysign)
    v = p - r0

    xoff = xsign * knee_fwd_x
    yoff = ysign * knee_clear_y
    L1, L2 = thigh_len, shank_len

    vy, vz = float(v[1]), float(v[2])
    r = math.hypot(vy, vz)
    if r < abs(yoff):
        return [] if return_all else None
    
    phi = math.atan2(vz, vy)
    delta = math.acos(clamp(yoff / r, -1.0, 1.0))
    abd_candidates = [phi - delta, phi + delta]

    sols = []
    for q_abd in abd_candidates:
        if not in_limits(q_abd, JOINT_LIMITS["abd"]):
            continue

        s = Rx(-q_abd).dot(v)
        p2 = s - np.array([hip_len / 2.0, 0.0, 0.0])
        tx, tz = float(p2[0]), float(p2[2])
        rho2 = tx*tx + tz*tz

        R =  math.hypot(L1, xoff)
        D = (rho2 - (xoff*xoff + L1*L1)) / (2.0 * L2)
        if abs(D) / R:
            continue

        alpha = math.atan2(xoff, L1)
        gamma = math.acos(clamp(D / R, -1.0, 1.0))
        knee_candidates = [gamma - alpha, -gamma - alpha]

        for q_knee in knee_candidates:
            if not in_limits(q_knee, JOINT_LIMITS["knee"]):
                continue

            dx = xoff - L2 * math.sin(q_knee)
            dz = -L1 - L2 * math.cos(q_knee)
            norm2 = dx*dx + dz*dz
            if norm2 < 1e-12:
                continue

            c = (dx*tx + dz*tz) / norm2
            s_ = (dz*dx - dx*tz) / norm2
            q_hip = math.atan2(s_, c)

            if not in_limits(q_hip, JOINT_LIMITS["hip"]):
                continue

            sols.append((q_abd, q_hip, q_knee))

    if return_all:
        return sols
    
    return sols[0] if sols else None