#!/usr/bin/env python3
"""Self-test for the evaluation harness -- runs without any dataset.

Validates the alignment maths, the quaternion conventions and the KITTI relative
errors against cases with analytically known answers, so a wrong number later can
be blamed on the SLAM run rather than on the evaluator.

    python3 evaluation/harness/selftest.py
"""

import os
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import metrics  # noqa: E402
import traj_io  # noqa: E402

FAILED = []


def check(name, cond, detail=""):
    status = "PASS" if cond else "FAIL"
    print("  [%s] %s%s" % (status, name, ("  -- " + detail) if detail else ""))
    if not cond:
        FAILED.append(name)


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def make_traj(n=500, radius=3.0, turns=1.5):
    """Helix-ish trajectory with non-trivial rotation at every pose."""
    t = np.linspace(0.0, turns * 2 * np.pi, n)
    xyz = np.column_stack([radius * np.cos(t), radius * np.sin(t), 0.3 * t])
    poses = np.tile(np.eye(4), (n, 1, 1))
    poses[:, :3, 3] = xyz
    for i in range(n):
        poses[i, :3, :3] = rot_z(t[i])
    stamps = np.arange(n) * 0.05
    return stamps, poses


def apply_sim3(poses, rot, trans, scale):
    out = poses.copy()
    out[:, :3, 3] = (scale * (rot @ poses[:, :3, 3].T)).T + trans
    for i in range(len(poses)):
        out[i, :3, :3] = rot @ poses[i, :3, :3]
    return out


print("== alignment ==")
_, gt = make_traj()

# A rigid transform of the trajectory must be fully absorbed by SE(3) alignment.
R = rot_z(0.7)
t = np.array([12.0, -4.0, 2.5])
est = apply_sim3(gt, R, t, 1.0)
r = metrics.ate(est, gt, align="se3")
check("SE3 recovers a rigid transform exactly", r["rmse"] < 1e-9,
      "rmse=%.3e m" % r["rmse"])

# A scaled trajectory: Sim(3) must recover the scale, SE(3) must NOT hide it.
est_scaled = apply_sim3(gt, R, t, 1.30)
r_sim3 = metrics.ate(est_scaled, gt, align="sim3")
r_se3 = metrics.ate(est_scaled, gt, align="se3")
check("Sim3 recovers the scale factor", abs(r_sim3["scale"] - 1.0 / 1.30) < 1e-9,
      "recovered s=%.6f, expected %.6f" % (r_sim3["scale"], 1.0 / 1.30))
check("Sim3 rmse ~0 on a purely scaled trajectory", r_sim3["rmse"] < 1e-9,
      "rmse=%.3e m" % r_sim3["rmse"])
check("SE3 exposes scale error instead of absorbing it", r_se3["rmse"] > 0.5,
      "rmse=%.4f m (this is the number stereo/RGB-D must report)" % r_se3["rmse"])
check("SE3 pins scale at exactly 1", r_se3["scale"] == 1.0)

# Known zero-mean perturbation: RMSE must match the injected magnitude.
rng = np.random.default_rng(0)
noise = rng.normal(0.0, 0.05, size=(len(gt), 3))
noise -= noise.mean(axis=0)
est_noisy = gt.copy()
est_noisy[:, :3, 3] += noise
expected = float(np.sqrt((noise ** 2).sum(axis=1).mean()))
r = metrics.ate(est_noisy, gt, align="se3")
check("rmse matches injected perturbation", abs(r["rmse"] - expected) < 0.01 * expected,
      "got %.6f, expected %.6f" % (r["rmse"], expected))

print("== quaternion conventions ==")
# ORB-SLAM3 writes (qx,qy,qz,qw); EuRoC ground truth uses (qw,qx,qy,qz).
ang = 0.9
Rz = rot_z(ang)
qw, qz = np.cos(ang / 2), np.sin(ang / 2)
tmp = tempfile.mkdtemp()

orb_file = os.path.join(tmp, "f_test.txt")
with open(orb_file, "w") as fh:  # ns timestamps, xyzw
    fh.write("1403715888384058112.0 1.0 2.0 3.0 0.0 0.0 %.17g %.17g\n" % (qz, qw))
    fh.write("1403715888434057984.0 1.1 2.0 3.0 0.0 0.0 %.17g %.17g\n" % (qz, qw))
    fh.write("1403715888484057984.0 1.2 2.0 3.0 0.0 0.0 %.17g %.17g\n" % (qz, qw))
s_orb, p_orb = traj_io.load_orb_euroc(orb_file)
check("ORB EuRoC ns timestamps -> seconds", abs(s_orb[0] - 1403715888.384058) < 1e-5,
      "t0=%.6f s" % s_orb[0])
check("ORB EuRoC quaternion read as xyzw", np.allclose(p_orb[0, :3, :3], Rz, atol=1e-12))

gt_file = os.path.join(tmp, "gt_test.txt")
with open(gt_file, "w") as fh:  # ns timestamps, wxyz, comma separated, ASL header
    fh.write("#timestamp [ns],p_RS_R_x [m],p_RS_R_y [m],p_RS_R_z [m],q_RS_w [],q_RS_x [],q_RS_y [],q_RS_z []\n")
    fh.write("1403715888384058112.0000000000,1.0,2.0,3.0,%.17g,0.0,0.0,%.17g\n" % (qw, qz))
    fh.write("1403715888434057984.0000000000,1.1,2.0,3.0,%.17g,0.0,0.0,%.17g\n" % (qw, qz))
s_gt, p_gt = traj_io.load_euroc_gt(gt_file)
check("EuRoC GT quaternion read as wxyz", np.allclose(p_gt[0, :3, :3], Rz, atol=1e-12))
check("both conventions land on the same rotation",
      np.allclose(p_orb[0, :3, :3], p_gt[0, :3, :3], atol=1e-12),
      "a swapped order here would silently rotate the whole trajectory")

print("== association ==")
ia, ib = traj_io.associate(s_orb, s_gt, max_diff=0.02)
check("associates only the overlapping poses", len(ia) == 2 and len(ib) == 2,
      "matched %d pairs of %d/%d" % (len(ia), len(s_orb), len(s_gt)))
ia2, _ = traj_io.associate(s_orb, s_gt + 10.0, max_diff=0.02)
check("rejects everything when clocks disagree", len(ia2) == 0)

print("== KITTI relative errors ==")
n = 1001
straight = np.tile(np.eye(4), (n, 1, 1))
straight[:, 0, 3] = np.arange(n, dtype=np.float64)  # 1 m steps along x, 1000 m total
scaled = straight.copy()
scaled[:, :3, 3] *= 1.05  # 5 % scale drift
k = metrics.kitti_relative(scaled, straight, lengths=(100, 200, 300, 400))
# The devkit overshoots each segment by up to one frame spacing and normalises by
# the nominal length, so the recovered value sits a hair above 5 %. See the
# convergence check below: the bias is discretisation, not a maths error.
check("t_rel recovers a 5% scale drift", abs(k["t_rel_pct"] - 5.0) < 0.05,
      "t_rel=%.4f %% (devkit discretisation bias %+.4f pp)" % (k["t_rel_pct"], k["t_rel_pct"] - 5.0))
check("r_rel ~0 when there is no rotation error", k["r_rel_deg_per_100m"] < 1e-9,
      "r_rel=%.3e deg/100m" % k["r_rel_deg_per_100m"])
check("path length measured on ground truth", abs(k["path_length_m"] - 1000.0) < 1e-9)

yawed = straight.copy()
drift = np.radians(1.0) / 100.0  # 1 deg per 100 m
for i in range(n):
    yawed[i, :3, :3] = rot_z(drift * i)
k2 = metrics.kitti_relative(yawed, straight, lengths=(100, 200, 300, 400))
check("r_rel recovers a 1 deg/100m yaw drift",
      abs(k2["r_rel_deg_per_100m"] - 1.0) < 0.01,
      "r_rel=%.4f deg/100m" % k2["r_rel_deg_per_100m"])

# The bias must shrink with finer frame spacing -- proves it is discretisation.
biases = []
for step_m, n in ((1.0, 1001), (0.25, 4001), (0.1, 10001)):
    s_tr = np.tile(np.eye(4), (n, 1, 1))
    s_tr[:, 0, 3] = np.arange(n) * step_m
    s_sc = s_tr.copy()
    s_sc[:, :3, 3] *= 1.05
    biases.append(metrics.kitti_relative(s_sc, s_tr, lengths=(100, 200, 300, 400))["t_rel_pct"] - 5.0)
check("discretisation bias converges to zero",
      biases[0] > biases[1] > biases[2] > 0 and biases[2] < 0.005,
      "bias %+.4f -> %+.4f -> %+.4f pp at 1.0 / 0.25 / 0.1 m spacing" % tuple(biases))

print("== aggregation ==")
agg = metrics.aggregate([0.030, 0.028, 0.035, 0.031, 0.029])
check("median is the headline statistic", abs(agg["median"] - 0.030) < 1e-12)
check("std reported for spread", agg["std"] > 0.0, "std=%.4f m" % agg["std"])
agg_nan = metrics.aggregate([0.03, None, float("nan"), 0.05])
check("failed runs dropped, not counted", agg_nan["n"] == 2)
check("empty input does not crash", metrics.aggregate([])["n"] == 0)

print("")
if FAILED:
    print("FAILED: %d check(s): %s" % (len(FAILED), ", ".join(FAILED)))
    sys.exit(1)
print("all checks passed")
