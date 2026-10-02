#!/usr/bin/env python3
"""Multi-sequence chains for run_benchmark.py / run_interleaved.py (registry: sequences_multi.yaml).

A chain is ONE process that tracks several sequences of the same scene one after the other
(Examples/Stereo/stereo_euroc and Examples/RGB-D/rgbd_tum accept N sequences and call System::ChangeDataset between
them), so ORB-SLAM3's Atlas has to merge the per-sequence maps itself. This is Zhang 2023, sec. 3.5 / 4.3 / Table VII.

Registry shape (see sequences_multi.yaml):
    elements:   {euroc: {MH01: {folder, times, gt}, ...}, tum: {...}}      the single sequences
    sequences:  {<arm>: {"MH01-02": {chain: [MH01, MH02]}, "MH02": {chain: [MH02]}, ...}}

What a run reports (all ATE are translation RMSE in metres after a rigid SE(3) alignment, as everywhere in this harness):
  ate_rmse (primary)      ATE of the LAST sequence of the chain, aligned on its own frames. This is the number of Table VII:
                          the text compares "MH01-02" with the single-sequence MH_02 of Table V.
  scores.last_mean_m      the same sequence, MEAN position error per frame (the thesis quotes "average ATE per frame" for TUM)
  scores.chain_se3        all frames of the chain with ONE alignment: it is only small if the merged map is consistent
  scores.chain_mean_m     mean error per frame of the same
  scores.last_in_chain_se3  error of the last sequence's frames under the alignment fitted on the WHOLE chain
  scores.seg_<name>       each sequence aligned on its own
  chain.maps_in_atlas     "There are N maps in the atlas" printed when the trajectory is saved
  chain.segments[...]     frames of the sequence / frames found in the saved trajectory

VALIDITY. SaveTrajectoryEuRoC writes only the frames of the BIGGEST map. A chain whose maps did not merge therefore loses
the frames of the smaller maps (and, when the first map is the biggest, the ATE of the last sequence is simply missing or
measured on a fragment). A run is INVALID (status "invalid_chain", dropped by stats_compare.py and listed in its flags,
counted by run_interleaved.py --summary) when
  * the log says N != 1 maps in the atlas (or has no such line; not asked of a one-sequence TUM run, which does not print it), or
  * a sequence has fewer than SEG_MIN_FRACTION (0.5) of its frames in the saved trajectory, or fewer than 3 matched poses.
0.5 is a judgement call, not a derived constant. It only has to catch a sequence that is essentially absent (its map was not the
biggest); the frames before the map initialisation are lost in healthy runs too (measured: the V102 part of a merged V101-V102 chain
kept 91% of its frames). maps_in_atlas == 1 is the primary test.
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)

SEG_MIN_FRACTION = 0.5
WINDOW_EPS_S = 1e-3


def is_chain(seq):
    return isinstance(seq, dict) and "chain" in seq


def _rpath(p):
    p = os.path.expanduser(str(p))
    return p if os.path.isabs(p) else os.path.join(REPO, p)


def elements_of(cfg, seq, reg):
    """[(name, element dict)] of a chain, from reg['elements'][dataset]."""
    pool = reg.get("elements", {}).get(cfg["dataset"], {})
    out = []
    for name in seq["chain"]:
        if name not in pool:
            raise KeyError("chain element %r is not in elements.%s of the registry" % (name, cfg["dataset"]))
        out.append((name, pool[name]))
    return out


def est_format(cfg, seq):
    """TUM chains of 2+ sequences are scored from the EuRoC-style file (biggest map only, ns timestamps); a TUM chain of
    ONE sequence runs the unchanged 4-argument rgbd_tum, which writes only CameraTrajectory.txt (all maps, seconds)."""
    if cfg["dataset"] == "tum" and len(seq["chain"]) == 1:
        return "orb_tum"
    return cfg["est_format"]


def _stamps_file(cfg, el, reg):
    if cfg["dataset"] == "euroc":
        return os.path.join(_rpath(cfg["timestamps_dir"]), el["times"]), 1e-9
    if cfg["dataset"] == "tum":
        return os.path.join(_rpath(cfg["associations_dir"]), el["assoc"]), 1.0
    raise KeyError("chains are supported for euroc and tum, not %r" % cfg["dataset"])


def read_stamps(cfg, el, reg):
    path, scale = _stamps_file(cfg, el, reg)
    out = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if line and not line.startswith("#"):
                out.append(float(line.split()[0]) * scale)
    return out


def duration_s(cfg, seq, reg):
    """Real-time playback length of the whole chain (sum over its sequences); None if a file is unreadable."""
    try:
        total = 0.0
        for _, el in elements_of(cfg, seq, reg):
            st = read_stamps(cfg, el, reg)
            total += st[-1] - st[0]
        return total
    except (OSError, ValueError, KeyError, IndexError):
        return None


def build_invocation(cfg, seq, reg, run_tag):
    """(argv, outputs, seq_dir) like run_benchmark.build_invocation; seq_dir is the first sequence's folder."""
    vocab = _rpath(reg["vocabulary"])
    root = _rpath(reg["roots"][cfg["dataset"]])
    settings = _rpath(seq.get("settings", cfg.get("settings")))
    els = elements_of(cfg, seq, reg)
    argv = [_rpath(cfg["binary"]), vocab, settings]
    for _, el in els:
        argv.append(os.path.join(root, el["folder"]))
        argv.append(_stamps_file(cfg, el, reg)[0])
    if cfg["dataset"] == "euroc":
        argv.append(run_tag)
        outputs = {"frames": "f_%s.txt" % run_tag, "keyframes": "kf_%s.txt" % run_tag}
    else:
        outputs = {"frames": "CameraTrajectory.txt" if len(els) == 1 else "CameraTrajectory_euroc.txt"}
    return argv, outputs, os.path.join(root, els[0][1]["folder"])


def gt_file(cfg, seq, reg):
    """Ground truth of the chain: the GT files of its sequences concatenated, in one file (timestamps are absolute and the
    sequences do not overlap in time; traj_io.associate sorts). Written once under ORB_CHAIN_GT_DIR
    (default <repo>/results_chain_gt), named after the dataset, GT source and chain."""
    import run_benchmark as rb
    root = _rpath(reg["roots"][cfg["dataset"]])
    parts = []
    for name, el in elements_of(cfg, seq, reg):
        parts.append(rb.resolve_gt(cfg, el, os.path.join(root, el["folder"]), reg))
    if len(parts) == 1:
        return parts[0]
    out_dir = os.environ.get("ORB_CHAIN_GT_DIR") or os.path.join(REPO, "results_chain_gt")
    tag = os.path.basename(os.path.dirname(parts[0])) if cfg["dataset"] == "euroc" else cfg["dataset"]
    out = os.path.join(out_dir, "%s_%s_%s.txt" % (cfg["dataset"], tag, "-".join(seq["chain"])))
    if all(os.path.exists(p) for p in parts):
        newest = max(os.path.getmtime(p) for p in parts)
        if not os.path.exists(out) or os.path.getmtime(out) < newest:
            os.makedirs(out_dir, exist_ok=True)
            tmp = out + ".tmp%d" % os.getpid()
            with open(tmp, "w") as dst:
                for p in parts:
                    with open(p) as src:
                        for line in src:
                            if line.strip() and not line.lstrip().startswith("#"):
                                dst.write(line if line.endswith("\n") else line + "\n")
            os.replace(tmp, out)
    return out


def missing_inputs(cfg, seq, reg):
    """Paths of the chain's inputs that do not exist (all sequences, not only the first)."""
    root = _rpath(reg["roots"][cfg["dataset"]])
    out = []
    for name, el in elements_of(cfg, seq, reg):
        for label, p in (("sequence", os.path.join(root, el["folder"])), ("timestamps/associations", _stamps_file(cfg, el, reg)[0])):
            if not os.path.exists(p):
                out.append("%s %s: %s" % (name, label, p))
    return out


def score(cfg, seq, reg, est_path, gt_path, maps_in_atlas=None, merges_detected=None):
    """Score one finished chain run. Returns the fields to merge into the run entry (see the module docstring)."""
    import numpy as np
    import metrics
    import traj_io

    els = elements_of(cfg, seq, reg)
    fmt = est_format(cfg, seq)
    es, ep = traj_io.load(est_path, fmt)
    gs, gp = traj_io.load(gt_path, cfg["gt_format"])
    ia, ib = traj_io.associate(es, gs, max_diff=cfg.get("max_diff", 0.02))
    if len(ia) < 3:
        raise ValueError("only %d pose pairs associated for the chain" % len(ia))
    e_xyz, g_xyz = ep[ia][:, :3, 3], gp[ib][:, :3, 3]
    t_assoc = es[ia]

    rot, trans, _ = metrics.umeyama(e_xyz, g_xyz, with_scale=False)
    err = np.linalg.norm((rot @ e_xyz.T).T + trans - g_xyz, axis=1)
    chain_ate = metrics.ate(ep[ia], gp[ib], align="se3")

    segments, scores, reasons = {}, {}, []
    last_name = els[-1][0]
    last_mask = None
    for name, el in els:
        st = read_stamps(cfg, el, reg)
        lo, hi = st[0] - WINDOW_EPS_S, st[-1] + WINDOW_EPS_S
        in_traj = int(((es >= lo) & (es <= hi)).sum())
        mask = (t_assoc >= lo) & (t_assoc <= hi)
        frac = in_traj / float(len(st))
        seg = {"frames": len(st), "in_trajectory": in_traj, "fraction": round(frac, 4), "matched": int(mask.sum()),
               "ate_rmse": None, "ate_mean": None}
        if mask.sum() >= 3:
            a = metrics.ate(ep[ia][mask], gp[ib][mask], align="se3")
            seg["ate_rmse"], seg["ate_mean"] = a["rmse"], a["mean"]
            scores["seg_%s" % name] = a["rmse"]
        else:
            reasons.append("sequence %s: %d matched poses (< 3)" % (name, int(mask.sum())))
        if frac < SEG_MIN_FRACTION:
            reasons.append("sequence %s: %.0f%% of its frames are in the saved trajectory (< %.0f%%)"
                           % (name, 100 * frac, 100 * SEG_MIN_FRACTION))
        segments[name] = seg
        if name == last_name:
            last_mask = mask

    last = segments[last_name]
    scores["last_se3"] = last["ate_rmse"]
    scores["last_mean_m"] = last["ate_mean"]
    scores["chain_se3"] = chain_ate["rmse"]
    scores["chain_mean_m"] = chain_ate["mean"]
    scores["last_in_chain_se3"] = float(np.sqrt((err[last_mask] ** 2).mean())) if last_mask is not None and last_mask.sum() >= 1 else None

    needs_maps = not (cfg["dataset"] == "tum" and len(els) == 1)
    if needs_maps:
        if maps_in_atlas is None:
            reasons.append("the log has no 'There are N maps in the atlas' line")
        elif maps_in_atlas != 1:
            reasons.append("maps_in_atlas=%d: the sequences did not all merge into one map" % maps_in_atlas)
    valid = not reasons
    return {
        "ate_rmse": last["ate_rmse"], "coverage": len(ia) / float(len(es)), "scores": scores,
        "chain": {"names": [n for n, _ in els], "valid": valid, "invalid_reasons": reasons,
                  "maps_in_atlas": maps_in_atlas, "merges_detected": merges_detected, "segments": segments,
                  "est_format": fmt, "min_fraction": SEG_MIN_FRACTION},
        "valid": valid,
    }


# ---------------------------------------------------------------------------------------
# self-test (needs numpy: run in the image)
# ---------------------------------------------------------------------------------------

def self_test():
    import tempfile
    import numpy as np

    fails = []

    def check(name, cond, info=""):
        print(("PASS " if cond else "FAIL ") + name + (("  " + info) if info else ""))
        if not cond:
            fails.append(name)

    tmp = tempfile.mkdtemp()
    os.environ["ORB_CHAIN_GT_DIR"] = os.path.join(tmp, "gt")
    os.makedirs(os.path.join(tmp, "ts"))
    reg = {"vocabulary": "v", "roots": {"euroc": os.path.join(tmp, "euroc")},
           "elements": {"euroc": {"A": {"folder": "A", "times": "A.txt", "gt": "A_GT.txt"},
                                  "B": {"folder": "B", "times": "B.txt", "gt": "B_GT.txt"}}}}
    cfg = {"dataset": "euroc", "sensor": "stereo", "align": "se3", "binary": "bin", "settings": "s.yaml",
           "timestamps_dir": os.path.join(tmp, "ts"), "est_format": "orb_euroc", "gt_format": "gt_euroc",
           "gt_dir": os.path.join(tmp, "gtsrc"), "gt_template": "{gt}", "max_diff": 0.02}
    os.makedirs(cfg["gt_dir"])
    rng = np.random.RandomState(0)

    def traj(t0, n):  # a smooth path, 20 Hz
        t = t0 + np.arange(n) / 20.0
        return t, np.column_stack([np.cos(t / 3.0) * 2, np.sin(t / 3.0) * 2, 0.1 * t])

    truth = {}
    for name, t0 in (("A", 1000.0), ("B", 5000.0)):
        t, xyz = traj(t0, 200)
        truth[name] = (t, xyz)
        with open(os.path.join(tmp, "ts", name + ".txt"), "w") as fh:
            fh.writelines("%d\n" % round(x * 1e9) for x in t)
        with open(os.path.join(cfg["gt_dir"], name + "_GT.txt"), "w") as fh:
            fh.writelines("%d,%f,%f,%f,1,0,0,0\n" % (round(a * 1e9), *p) for a, p in zip(t, xyz))

    def write_est(path, drop_b=0.0, noise=0.01):
        with open(path, "w") as fh:
            for name in ("A", "B"):
                t, xyz = truth[name]
                for i, (a, p) in enumerate(zip(t, xyz)):
                    if name == "B" and i < drop_b * len(t):
                        continue
                    q = p + rng.normal(0, noise, 3)
                    fh.write("%.6f %.9f %.9f %.9f 0 0 0 1\n" % (a * 1e9, *q))

    seq = {"chain": ["A", "B"]}
    gt = gt_file(cfg, seq, reg)
    check("concatenated GT has both sequences", sum(1 for _ in open(gt)) == 400)
    argv, outputs, sdir = build_invocation(cfg, seq, reg, "run00")
    check("euroc argv: bin vocab settings (dir times)xN tag", len(argv) == 3 + 4 + 1 and argv[-1] == "run00"
          and outputs["frames"] == "f_run00.txt")
    est = os.path.join(tmp, "f.txt")
    write_est(est)
    r = score(cfg, seq, reg, est, gt, maps_in_atlas=1)
    check("merged chain is valid, ATE about the noise", r["valid"] and r["ate_rmse"] < 0.05 and r["scores"]["chain_se3"] < 0.05,
          "ate %.4f chain %.4f" % (r["ate_rmse"], r["scores"]["chain_se3"]))
    r = score(cfg, seq, reg, est, gt, maps_in_atlas=2)
    check("two maps in the atlas: INVALID", not r["valid"] and "maps_in_atlas=2" in r["chain"]["invalid_reasons"][0])
    r = score(cfg, seq, reg, est, gt, maps_in_atlas=None)
    check("no atlas line: INVALID", not r["valid"])
    write_est(est, drop_b=0.7)
    r = score(cfg, seq, reg, est, gt, maps_in_atlas=1)
    check("70% of the last sequence missing from the trajectory: INVALID", not r["valid"]
          and any("sequence B" in x for x in r["chain"]["invalid_reasons"]))
    check("duration is the sum of the sequences", abs(duration_s(cfg, seq, reg) - 2 * 199 / 20.0) < 1e-6)
    print("chain_eval self-test: %s" % ("FAILED: " + "; ".join(fails) if fails else "ALL PASSED"))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
