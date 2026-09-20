#!/usr/bin/env python3
"""Score one estimated trajectory against ground truth.

Replaces evaluation/evaluate_ate_scale.py, which is Python 2 (does not run on
this machine) and always solves for scale.

    python3 evaluation/harness/evaluate.py \
        --est f_v103_stereo.txt --est-format orb_euroc \
        --gt evaluation/Ground_truth/EuRoC_left_cam/V103_GT.txt --gt-format gt_euroc \
        --align se3
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import metrics  # noqa: E402
import traj_io  # noqa: E402


def evaluate(est_path, gt_path, est_format, gt_format, align="se3",
             max_diff=0.02, offset=0.0, kitti=False):
    est_stamps, est_poses = traj_io.load(est_path, est_format)
    gt_stamps, gt_poses = traj_io.load(gt_path, gt_format)

    ia, ib = traj_io.associate(est_stamps, gt_stamps, max_diff=max_diff, offset=offset)
    if len(ia) < 3:
        raise ValueError(
            "only %d pose pairs associated (est %d poses spanning %.1f s, gt %d poses "
            "spanning %.1f s, max_diff %.3f s) -- check the timestamp units and that "
            "the trajectory is not from a different sequence"
            % (len(ia), len(est_stamps), est_stamps[-1] - est_stamps[0],
               len(gt_stamps), gt_stamps[-1] - gt_stamps[0], max_diff))

    est_m, gt_m = est_poses[ia], gt_poses[ib]
    out = {
        "est": est_path,
        "gt": gt_path,
        "ate": metrics.ate(est_m, gt_m, align=align),
        "coverage": float(len(ia)) / float(len(gt_stamps)),
        "est_poses": int(len(est_stamps)),
        "gt_poses": int(len(gt_stamps)),
    }
    if kitti:
        out["kitti"] = metrics.kitti_relative(est_m, gt_m)
    return out


def format_report(r):
    a = r["ate"]
    lines = [
        "trajectory : %s" % r["est"],
        "ground truth: %s" % r["gt"],
        "associated : %d pairs  (%.1f%% of ground truth; %d estimated poses)"
        % (a["n"], 100.0 * r["coverage"], r["est_poses"]),
        "alignment  : %s%s" % (a["align"],
                               ("  (scale=%.6f)" % a["scale"]) if a["align"] == "sim3" else ""),
        "",
        "ATE RMSE   : %.4f m" % a["rmse"],
        "    mean %.4f  median %.4f  std %.4f  min %.4f  max %.4f"
        % (a["mean"], a["median"], a["std"], a["min"], a["max"]),
    ]
    if "kitti" in r:
        k = r["kitti"]
        lines += [
            "",
            "KITTI t_rel: %.4f %%      r_rel: %.4f deg/100m   (path %.1f m)"
            % (k["t_rel_pct"], k["r_rel_deg_per_100m"], k["path_length_m"]),
        ]
    if r["coverage"] < 0.7:
        lines += ["",
                  "WARNING: only %.1f%% of the ground truth was matched. Tracking most "
                  "likely got lost -- a low ATE on a partial trajectory is not a good "
                  "result." % (100.0 * r["coverage"])]
    return "\n".join(lines)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--est", required=True)
    p.add_argument("--gt", required=True)
    p.add_argument("--est-format", required=True, choices=sorted(traj_io.LOADERS))
    p.add_argument("--gt-format", required=True, choices=sorted(traj_io.LOADERS))
    p.add_argument("--align", default="se3", choices=["se3", "sim3"],
                   help="se3 for stereo/RGB-D/inertial (default), sim3 only for monocular")
    p.add_argument("--max-diff", type=float, default=0.02,
                   help="max timestamp difference in s (use 0.5 for frame-indexed KITTI)")
    p.add_argument("--offset", type=float, default=0.0)
    p.add_argument("--kitti", action="store_true", help="also report t_rel / r_rel")
    p.add_argument("--json", action="store_true")
    a = p.parse_args()

    r = evaluate(a.est, a.gt, a.est_format, a.gt_format, align=a.align,
                 max_diff=a.max_diff, offset=a.offset, kitti=a.kitti)
    print(json.dumps(r, indent=2) if a.json else format_report(r))


if __name__ == "__main__":
    main()
