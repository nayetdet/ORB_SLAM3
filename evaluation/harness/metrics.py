"""Trajectory error metrics: ATE (SE3/Sim3 aligned) and the KITTI relative errors.

numpy only -- no evo, no scipy. The old evaluation/evaluate_ate_scale.py is
Python 2 and always solves for scale, which is wrong for stereo and RGB-D:
a scale-corrected RMSE hides scale drift and is not comparable with the numbers
published for ORB-SLAM2/3. Use align="se3" for anything but monocular.
"""

import numpy as np


def umeyama(src, dst, with_scale=False):
    """Least-squares similarity transform mapping src onto dst.

    src, dst: (N,3). Returns (R, t, s) minimising ||s*R*src_i + t - dst_i||^2.
    With with_scale=False the scale is fixed at 1, i.e. a rigid SE(3) alignment.
    """
    src = np.asarray(src, dtype=np.float64)
    dst = np.asarray(dst, dtype=np.float64)
    n = len(src)
    mu_src, mu_dst = src.mean(axis=0), dst.mean(axis=0)
    sc, dc = src - mu_src, dst - mu_dst
    cov = dc.T @ sc / n
    u, d, vt = np.linalg.svd(cov)
    s_mat = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s_mat[2, 2] = -1.0
    rot = u @ s_mat @ vt
    if with_scale:
        var_src = (sc ** 2).sum() / n
        scale = float(np.trace(np.diag(d) @ s_mat) / var_src) if var_src > 1e-12 else 1.0
    else:
        scale = 1.0
    trans = mu_dst - scale * rot @ mu_src
    return rot, trans, scale


def _rot_angle(rot):
    """Geodesic angle of a rotation matrix, in radians."""
    c = (np.trace(rot) - 1.0) * 0.5
    return float(np.arccos(np.clip(c, -1.0, 1.0)))


def ate(est_poses, gt_poses, align="se3"):
    """Absolute trajectory error between associated pose arrays.

    est_poses, gt_poses: (N,4,4), already associated one-to-one.
    align: "se3" (rigid, scale fixed to 1) or "sim3" (solves for scale, monocular).

    Returns a dict with rmse/mean/median/std/min/max of the translation error in
    metres, the recovered scale, and the pose count.
    """
    if align not in ("se3", "sim3"):
        raise ValueError("align must be 'se3' or 'sim3', got %r" % align)
    if len(est_poses) != len(gt_poses):
        raise ValueError("pose count mismatch: %d vs %d" % (len(est_poses), len(gt_poses)))
    if len(est_poses) < 3:
        raise ValueError("need at least 3 associated poses, got %d" % len(est_poses))

    est_xyz = est_poses[:, :3, 3]
    gt_xyz = gt_poses[:, :3, 3]
    rot, trans, scale = umeyama(est_xyz, gt_xyz, with_scale=(align == "sim3"))
    aligned = (scale * (rot @ est_xyz.T)).T + trans
    err = np.linalg.norm(aligned - gt_xyz, axis=1)
    return {
        "rmse": float(np.sqrt((err ** 2).mean())),
        "mean": float(err.mean()),
        "median": float(np.median(err)),
        "std": float(err.std()),
        "min": float(err.min()),
        "max": float(err.max()),
        "scale": scale,
        "n": int(len(err)),
        "align": align,
    }


def _trajectory_distances(poses):
    d = [0.0]
    for i in range(1, len(poses)):
        d.append(d[-1] + float(np.linalg.norm(poses[i, :3, 3] - poses[i - 1, :3, 3])))
    return np.asarray(d)


def _last_frame_from_length(dists, first, length):
    for i in range(first, len(dists)):
        if dists[i] > dists[first] + length:
            return i
    return -1


def kitti_relative(est_poses, gt_poses, lengths=(100, 200, 300, 400, 500, 600, 700, 800),
                   step=10):
    """KITTI odometry metrics: translation error [%] and rotation error [deg/m].

    Reimplements the official devkit: for every step-th start frame and every
    segment length, compare the relative pose over that segment and normalise by
    the segment length. This is the metric KITTI is actually scored on -- ATE in
    metres on a 3 km sequence is dominated by a single far-end drift and is not
    what the ORB-SLAM2/3 papers report for this dataset.

    Faithful to the devkit, including its discretisation bias: the segment ends at
    the first frame *past* the nominal length but the error is normalised by the
    nominal length, so a segment overshoots by up to one frame spacing. On KITTI
    (~1 m between frames, shortest segment 100 m) that inflates t_rel by <0.05 pp.
    Kept deliberately -- matching the published ORB-SLAM2/3 numbers matters more
    than removing a bias that is present in theirs too.

    Returns dict with t_rel [%], r_rel [deg/100m] and the per-length breakdown.
    """
    if len(est_poses) != len(gt_poses):
        raise ValueError("pose count mismatch: %d vs %d" % (len(est_poses), len(gt_poses)))
    dists = _trajectory_distances(gt_poses)
    t_errs, r_errs, per_len = [], [], {}
    for length in lengths:
        lt, lr = [], []
        for first in range(0, len(gt_poses), step):
            last = _last_frame_from_length(dists, first, length)
            if last == -1:
                continue
            gt_delta = np.linalg.inv(gt_poses[first]) @ gt_poses[last]
            es_delta = np.linalg.inv(est_poses[first]) @ est_poses[last]
            err = np.linalg.inv(es_delta) @ gt_delta
            lt.append(float(np.linalg.norm(err[:3, 3])) / length)
            lr.append(_rot_angle(err[:3, :3]) / length)
        if lt:
            per_len[length] = {
                "t_rel_pct": float(np.mean(lt) * 100.0),
                "r_rel_deg_per_100m": float(np.degrees(np.mean(lr)) * 100.0),
                "n": len(lt),
            }
            t_errs.extend(lt)
            r_errs.extend(lr)
    if not t_errs:
        raise ValueError("sequence too short for lengths %s (total %.1f m)"
                         % (list(lengths), dists[-1]))
    return {
        "t_rel_pct": float(np.mean(t_errs) * 100.0),
        "r_rel_deg_per_100m": float(np.degrees(np.mean(r_errs)) * 100.0),
        "path_length_m": float(dists[-1]),
        "per_length": per_len,
    }


def aggregate(values):
    """Summarise a metric across N runs. Median is the headline, not the mean.

    ORB-SLAM3 is multi-threaded and non-deterministic; the run-to-run spread on
    EuRoC is comparable to the differences the thesis reports as improvements.
    The ORB-SLAM3 paper reports the median of 10 executions for this reason.
    """
    v = np.asarray([x for x in values if x is not None and np.isfinite(x)], dtype=np.float64)
    if len(v) == 0:
        return {"median": None, "mean": None, "std": None, "min": None,
                "max": None, "n": 0}
    return {
        "median": float(np.median(v)),
        "mean": float(v.mean()),
        "std": float(v.std(ddof=1)) if len(v) > 1 else 0.0,
        "min": float(v.min()),
        "max": float(v.max()),
        "n": int(len(v)),
    }
