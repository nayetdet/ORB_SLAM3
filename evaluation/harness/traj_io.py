"""Trajectory loaders for ORB-SLAM3 output and dataset ground truth.

Every loader returns (stamps, poses):
    stamps : (N,)   float64, seconds
    poses  : (N,4,4) float64, T_world_body (body = whatever frame the file is in)

Frame/format gotchas this module absorbs:
  * SaveTrajectoryEuRoC writes timestamps as 1e9*t (nanoseconds); SaveTrajectoryTUM
    writes seconds. Both are normalised to seconds here.
  * ORB-SLAM3 writes quaternions as (qx,qy,qz,qw); the EuRoC/ASL ground truth uses
    (qw,qx,qy,qz). Mixing them up rotates the trajectory and silently inflates ATE.
  * SaveTrajectoryEuRoC emits the IMU/body pose for inertial sensors and the cam0
    pose otherwise, which is why the repo ships both Ground_truth/EuRoC_imu and
    Ground_truth/EuRoC_left_cam. Pick the matching one -- see sequences.yaml.
"""

import numpy as np


def _quat_to_rot(q):
    """Unit quaternion (w,x,y,z) -> 3x3 rotation matrix."""
    w, x, y, z = q
    n = w * w + x * x + y * y + z * z
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def _poses_from_quat(xyz, quat_wxyz):
    n = len(xyz)
    poses = np.tile(np.eye(4), (n, 1, 1))
    poses[:, :3, 3] = xyz
    for i in range(n):
        poses[i, :3, :3] = _quat_to_rot(quat_wxyz[i])
    return poses


def _read_numeric_rows(path, sep=None):
    rows = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line[0] == "#":
                continue
            parts = line.replace(",", " ").split() if sep is None else line.split(sep)
            try:
                rows.append([float(p) for p in parts if p != ""])
            except ValueError:
                continue  # header line that did not start with '#'
    if not rows:
        raise ValueError("no numeric rows in %s" % path)
    width = len(rows[0])
    rows = [r for r in rows if len(r) == width]
    return np.asarray(rows, dtype=np.float64)


def load_tum(path, stamp_scale=1.0):
    """TUM format: ts tx ty tz qx qy qz qw  (ORB-SLAM3 SaveTrajectoryTUM / EuRoC).

    stamp_scale converts the file's timestamp unit to seconds
    (1.0 for SaveTrajectoryTUM, 1e-9 for SaveTrajectoryEuRoC).
    """
    d = _read_numeric_rows(path)
    if d.shape[1] < 8:
        raise ValueError("%s: expected 8 columns (ts tx ty tz qx qy qz qw), got %d"
                         % (path, d.shape[1]))
    stamps = d[:, 0] * stamp_scale
    xyz = d[:, 1:4]
    quat_wxyz = np.column_stack([d[:, 7], d[:, 4], d[:, 5], d[:, 6]])  # xyzw -> wxyz
    return stamps, _poses_from_quat(xyz, quat_wxyz)


def load_orb_euroc(path):
    """ORB-SLAM3 f_*.txt / kf_*.txt from SaveTrajectoryEuRoC (timestamps in ns)."""
    return load_tum(path, stamp_scale=1e-9)


def load_euroc_gt(path):
    """EuRoC/ASL ground truth: ts[ns], px,py,pz, qw,qx,qy,qz [, ...].

    Covers both evaluation/Ground_truth/EuRoC_left_cam/*_GT.txt (8 columns) and
    EuRoC_imu/*_GT.txt / the dataset's own state_groundtruth_estimate0/data.csv
    (17 columns; the extra velocity and bias columns are ignored).
    """
    d = _read_numeric_rows(path)
    if d.shape[1] < 8:
        raise ValueError("%s: expected >=8 columns, got %d" % (path, d.shape[1]))
    stamps = d[:, 0] * 1e-9
    xyz = d[:, 1:4]
    quat_wxyz = d[:, 4:8]  # already w,x,y,z
    return stamps, _poses_from_quat(xyz, quat_wxyz)


def load_kitti_poses(path, times_path=None):
    """KITTI format: 12 floats per line = 3x4 row-major T_world_cam, no timestamp.

    Matches both the dataset's poses/XX.txt and ORB-SLAM3's SaveTrajectoryKITTI.
    Frame index is used as the association key unless times_path is given.
    """
    d = _read_numeric_rows(path)
    if d.shape[1] != 12:
        raise ValueError("%s: expected 12 columns, got %d" % (path, d.shape[1]))
    n = len(d)
    poses = np.tile(np.eye(4), (n, 1, 1))
    poses[:, :3, :4] = d.reshape(n, 3, 4)
    if times_path is not None:
        stamps = _read_numeric_rows(times_path)[:, 0]
        if len(stamps) < n:
            raise ValueError("%s has %d stamps for %d poses" % (times_path, len(stamps), n))
        stamps = stamps[:n]
    else:
        stamps = np.arange(n, dtype=np.float64)
    return stamps, poses


def load_tum_rgbd_gt(path):
    """TUM RGB-D groundtruth.txt: ts tx ty tz qx qy qz qw, timestamps in seconds."""
    return load_tum(path, stamp_scale=1.0)


LOADERS = {
    "orb_euroc": load_orb_euroc,
    "orb_tum": load_tum,
    "orb_kitti": load_kitti_poses,
    "gt_euroc": load_euroc_gt,
    "gt_tum": load_tum_rgbd_gt,
    "gt_kitti": load_kitti_poses,
}


def load(path, fmt, **kw):
    if fmt not in LOADERS:
        raise KeyError("unknown format %r (known: %s)" % (fmt, ", ".join(sorted(LOADERS))))
    return LOADERS[fmt](path, **kw)


def associate(stamps_a, stamps_b, max_diff=0.02, offset=0.0):
    """Nearest-neighbour timestamp matching, each pose used at most once.

    Returns (idx_a, idx_b) index arrays. max_diff is in seconds; for
    index-keyed KITTI trajectories pass max_diff=0.5 so frame i matches frame i.
    """
    b_shift = stamps_b + offset
    order = np.argsort(b_shift)
    b_sorted = b_shift[order]
    idx_a, idx_b = [], []
    used = np.zeros(len(b_sorted), dtype=bool)
    for i, ta in enumerate(stamps_a):
        j = int(np.searchsorted(b_sorted, ta))
        best, best_d = -1, np.inf
        for cand in (j - 1, j, j + 1):
            if 0 <= cand < len(b_sorted) and not used[cand]:
                d = abs(b_sorted[cand] - ta)
                if d < best_d:
                    best, best_d = cand, d
        if best >= 0 and best_d <= max_diff:
            used[best] = True
            idx_a.append(i)
            idx_b.append(int(order[best]))
    return np.asarray(idx_a, dtype=int), np.asarray(idx_b, dtype=int)
