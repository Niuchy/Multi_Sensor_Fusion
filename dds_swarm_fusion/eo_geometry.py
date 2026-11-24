"""Guarded two-station bearing intersection, also usable without elevation."""
import numpy as np


def intersect(p1, b1, p2, b2, skew_s=0.0):
    p1, p2 = np.asarray(p1, float), np.asarray(p2, float)
    if np.linalg.norm(p2-p1) < 5.0:
        return None, "baseline_too_short"
    three_d = b1.get("el") is not None and b2.get("el") is not None
    angles = [float(b1["az"]), float(b2["az"])]
    sigmas = [float(b1["sig_az"]), float(b2["sig_az"])]
    if three_d:
        angles += [float(b1["el"]), float(b2["el"])]
        sigmas += [float(b1.get("sig_el", b1["sig_az"])),
                   float(b2.get("sig_el", b2["sig_az"]))]
    else:
        p1, p2 = p1[:2], p2[:2]

    def solve(a):
        if three_d:
            u = np.array([np.cos(a[2])*np.cos(a[0]), np.cos(a[2])*np.sin(a[0]), np.sin(a[2])])
            v = np.array([np.cos(a[3])*np.cos(a[1]), np.cos(a[3])*np.sin(a[1]), np.sin(a[3])])
        else:
            u, v = np.array([np.cos(a[0]), np.sin(a[0])]), np.array([np.cos(a[1]), np.sin(a[1])])
        A = np.column_stack([u, -v])
        distances = np.linalg.lstsq(A, p2-p1, rcond=None)[0]
        q1, q2 = p1 + distances[0]*u, p2 + distances[1]*v
        return (q1+q2)/2, distances, np.linalg.norm(q1-q2), abs(float(u@v))

    pt, distances, miss, cosine = solve(angles)
    angle = np.arccos(np.clip(cosine, 0, 1))
    if angle < max(np.deg2rad(5), 3*np.hypot(sigmas[0], sigmas[1])):
        return None, "intersection_angle_too_small"
    if np.min(distances) <= 0 or np.max(distances) > 30000:
        return None, "intersection_behind_or_out_of_range"
    uncertainty = np.hypot(distances[0]*sigmas[0], distances[1]*sigmas[1])
    motion_sigma = 120.0 * abs(float(skew_s))
    if miss > max(10.0, 3.0*uncertainty + motion_sigma):
        return None, "rays_do_not_meet"
    eps = 1e-5
    J = []
    for i in range(len(angles)):
        ap, am = np.array(angles), np.array(angles)
        ap[i] += eps
        am[i] -= eps
        J.append((solve(ap)[0]-solve(am)[0])/(2*eps))
    J = np.column_stack(J)
    cov = J @ np.diag(np.square(sigmas)) @ J.T
    cov += np.eye(len(pt)) * (1.0 + motion_sigma**2)
    sigma_max = np.sqrt(max(float(np.max(np.linalg.eigvalsh(cov))), 0))
    if not np.all(np.isfinite(pt)) or sigma_max > 50.0:
        return None, "intersection_uncertainty_too_large"
    return dict(pt=pt[:2], cov=cov[:2, :2],
                cross_xy_alt=cov[:2, 2].copy() if three_d else None,
                alt=float(pt[2]) if three_d else None,
                alt_sigma=float(np.sqrt(cov[2, 2])) if three_d else None,
                sigma_max_m=sigma_max, ray_miss_m=float(miss)), "accepted"
