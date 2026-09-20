#!/usr/bin/env python3
"""Run a SLAM config N times over a set of sequences and report median metrics.

Why N runs: ORB-SLAM3 is multi-threaded and non-deterministic. On EuRoC the
run-to-run spread is of the same order as the differences the Zhang 2023 thesis
reports as improvements (e.g. MH_02 0.028 -> 0.026 m), so a 3-run mean cannot
tell a real change from noise. The ORB-SLAM3 paper reports the median of 10.

    # validate paths and print the commands without running anything
    python3 evaluation/harness/run_benchmark.py --config euroc_stereo --dry-run

    # the real thing
    python3 evaluation/harness/run_benchmark.py \
        --config euroc_stereo --sequences MH04,V103 --runs 10 --tag baseline

    # compare two result sets
    python3 evaluation/harness/run_benchmark.py --compare results/baseline results/inertial
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import evaluate as ev  # noqa: E402
import metrics  # noqa: E402

try:
    import yaml
except ImportError:
    sys.exit("PyYAML is required: pip install pyyaml  (or apt install python3-yaml)")

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))


def rpath(p):
    """Resolve a path from sequences.yaml against the repo root."""
    p = os.path.expanduser(str(p))
    return p if os.path.isabs(p) else os.path.join(REPO, p)


def load_registry(path=None):
    with open(path or os.path.join(HERE, "sequences.yaml")) as fh:
        return yaml.safe_load(fh)


def build_invocation(cfg_name, cfg, seq_name, seq, reg, run_tag):
    """Return (argv, output_map) where output_map maps kind -> filename in cwd."""
    vocab = rpath(reg["vocabulary"])
    root = rpath(reg["roots"][cfg["dataset"]])
    seq_dir = os.path.join(root, seq["folder"])
    settings = rpath(seq.get("settings", cfg.get("settings")))

    if cfg["dataset"] == "euroc":
        times = os.path.join(rpath(cfg["timestamps_dir"]), seq["times"])
        argv = [rpath(cfg["binary"]), vocab, settings, seq_dir, times, run_tag]
        outputs = {"frames": "f_%s.txt" % run_tag, "keyframes": "kf_%s.txt" % run_tag}
    elif cfg["dataset"] == "kitti":
        argv = [rpath(cfg["binary"]), vocab, settings, seq_dir]
        outputs = {"frames": "CameraTrajectory.txt"}
    elif cfg["dataset"] == "tum":
        assoc = os.path.join(rpath(cfg["associations_dir"]), seq["assoc"])
        argv = [rpath(cfg["binary"]), vocab, settings, seq_dir, assoc]
        outputs = {"frames": "CameraTrajectory.txt",
                   "keyframes": "KeyFrameTrajectory.txt"}
    else:
        raise KeyError("unsupported dataset %r" % cfg["dataset"])
    return argv, outputs, seq_dir


def resolve_gt(cfg, seq, seq_dir, reg):
    if "gt_from_sequence" in cfg:
        return os.path.join(seq_dir, cfg["gt_from_sequence"])
    root = rpath(reg["roots"][cfg["dataset"]])
    tmpl = cfg.get("gt_template", "{gt}")
    rel = tmpl.format(gt=seq["gt"])
    if "gt_dir" in cfg:
        return os.path.join(rpath(cfg["gt_dir"]), rel)
    return os.path.join(root, rel)


def run_one(argv, cwd, log_path, timeout):
    t0 = time.time()
    with open(log_path, "w") as log:
        try:
            rc = subprocess.call(argv, cwd=cwd, stdout=log, stderr=subprocess.STDOUT,
                                 timeout=timeout)
        except subprocess.TimeoutExpired:
            log.write("\n*** TIMEOUT after %s s ***\n" % timeout)
            rc = -9
        except OSError as exc:
            log.write("\n*** could not execute: %s ***\n" % exc)
            rc = -1
    return rc, time.time() - t0


def benchmark(args):
    reg = load_registry(args.registry)
    cfg_name = args.config
    if cfg_name not in reg["configs"]:
        sys.exit("unknown config %r (known: %s)" % (cfg_name, ", ".join(reg["configs"])))
    cfg = reg["configs"][cfg_name]
    all_seqs = reg["sequences"][cfg_name]
    names = args.sequences.split(",") if args.sequences else list(all_seqs)
    for n in names:
        if n not in all_seqs:
            sys.exit("unknown sequence %r for %s (known: %s)"
                     % (n, cfg_name, ", ".join(all_seqs)))

    tag = args.tag or cfg_name
    out_dir = rpath(os.path.join(args.out, tag))
    if not args.dry_run:
        os.makedirs(out_dir, exist_ok=True)

    report = {
        "config": cfg_name,
        "tag": tag,
        "runs": args.runs,
        "trajectory": args.trajectory,
        "align": cfg["align"],
        "sensor": cfg["sensor"],
        "started": time.strftime("%Y-%m-%d %H:%M:%S"),
        "host_cpu_count": os.cpu_count(),
        "sequences": {},
    }

    missing = []
    for seq_name in names:
        seq = all_seqs[seq_name]
        argv, outputs, seq_dir = build_invocation(cfg_name, cfg, seq_name, seq, reg, "run")
        gt_path = resolve_gt(cfg, seq, seq_dir, reg)

        if args.trajectory not in outputs:
            sys.exit("config %s does not produce a %r trajectory (has: %s)"
                     % (cfg_name, args.trajectory, ", ".join(outputs)))

        for label, p in (("binary", argv[0]), ("vocabulary", argv[1]),
                         ("settings", argv[2]), ("sequence", seq_dir),
                         ("ground truth", gt_path)):
            if not os.path.exists(p):
                missing.append("%s: %s not found -> %s" % (seq_name, label, p))

        if args.dry_run:
            print("[%s] %s" % (seq_name, " ".join(argv)))
            print("        gt: %s" % gt_path)
            print("        scoring: %s trajectory, %s alignment"
                  % (args.trajectory, cfg["align"]))
            continue

        seq_out = os.path.join(out_dir, seq_name)
        os.makedirs(seq_out, exist_ok=True)
        runs = []
        for i in range(args.runs):
            run_tag = "run%02d" % i
            argv_i, outputs_i, _ = build_invocation(cfg_name, cfg, seq_name, seq, reg, run_tag)
            work = os.path.join(seq_out, run_tag)
            os.makedirs(work, exist_ok=True)
            log_path = os.path.join(work, "slam.log")
            print("  %s %s ... " % (seq_name, run_tag), end="", flush=True)
            rc, elapsed = run_one(argv_i, work, log_path, args.timeout)

            src = os.path.join(work, outputs_i[args.trajectory])
            entry = {"run": run_tag, "returncode": rc, "seconds": round(elapsed, 1),
                     "log": os.path.relpath(log_path, REPO)}
            if rc != 0 or not os.path.exists(src):
                entry["status"] = "failed"
                entry["ate_rmse"] = None
                print("FAILED (rc=%d, %.0fs) -- see %s" % (rc, elapsed, entry["log"]))
                runs.append(entry)
                continue

            traj = os.path.join(seq_out, "%s_%s.txt" % (args.trajectory, run_tag))
            shutil.move(src, traj)
            entry["trajectory"] = os.path.relpath(traj, REPO)
            try:
                r = ev.evaluate(traj, gt_path, cfg["est_format"], cfg["gt_format"],
                                align=cfg["align"], max_diff=cfg.get("max_diff", 0.02),
                                kitti=bool(cfg.get("kitti_relative")))
                entry["status"] = "ok"
                entry["ate_rmse"] = r["ate"]["rmse"]
                entry["coverage"] = r["coverage"]
                if "kitti" in r:
                    entry["t_rel_pct"] = r["kitti"]["t_rel_pct"]
                    entry["r_rel_deg_per_100m"] = r["kitti"]["r_rel_deg_per_100m"]
                msg = "ATE %.4f m (cov %.0f%%, %.0fs)" % (
                    r["ate"]["rmse"], 100 * r["coverage"], elapsed)
                if "kitti" in r:
                    msg += "  t_rel %.2f%%" % r["kitti"]["t_rel_pct"]
                print(msg)
            except Exception as exc:  # evaluation failure is a result, not a crash
                entry["status"] = "eval_error"
                entry["ate_rmse"] = None
                entry["error"] = str(exc)
                print("EVAL ERROR: %s" % exc)
            runs.append(entry)

        seq_report = {
            "runs": runs,
            "ate_rmse": metrics.aggregate([r.get("ate_rmse") for r in runs]),
            "failures": sum(1 for r in runs if r["status"] != "ok"),
        }
        if cfg.get("kitti_relative"):
            seq_report["t_rel_pct"] = metrics.aggregate([r.get("t_rel_pct") for r in runs])
            seq_report["r_rel_deg_per_100m"] = metrics.aggregate(
                [r.get("r_rel_deg_per_100m") for r in runs])
        report["sequences"][seq_name] = seq_report

    if missing:
        print("\nMissing inputs:")
        for m in missing:
            print("  " + m)
        if not args.dry_run:
            sys.exit(1)
    if args.dry_run:
        print("\ndry run: nothing executed. %d sequence(s) checked, %d missing input(s)."
              % (len(names), len(missing)))
        return

    json_path = os.path.join(out_dir, "results.json")
    with open(json_path, "w") as fh:
        json.dump(report, fh, indent=2)
    md = render_table(report, reg)
    with open(os.path.join(out_dir, "results.md"), "w") as fh:
        fh.write(md)
    print("\n" + md)
    print("written: %s" % os.path.relpath(json_path, REPO))


def render_table(report, reg=None):
    cfg_name = report["config"]
    kitti = any("t_rel_pct" in s for s in report["sequences"].values())
    lines = [
        "## %s  (tag: %s)" % (cfg_name, report["tag"]),
        "",
        "- runs per sequence: **%d**, reported as median" % report["runs"],
        "- trajectory: `%s`, alignment: `%s`, sensor: `%s`"
        % (report["trajectory"], report["align"], report["sensor"]),
        "- host: %s logical CPUs, started %s" % (report["host_cpu_count"], report["started"]),
        "",
    ]
    ref = None
    if reg and cfg_name in reg.get("reference", {}):
        ref = reg["reference"][cfg_name]

    head = "| seq | ATE median [m] | std | min | max | ok/N |"
    sep = "|---|---|---|---|---|---|"
    if kitti:
        head += " t_rel [%] | r_rel [deg/100m] |"
        sep += "---|---|"
    if ref:
        head += " ref [m] | Δ |"
        sep += "---|---|"
    lines += [head, sep]

    for name, s in report["sequences"].items():
        a = s["ate_rmse"]
        n_ok = a["n"]
        if n_ok == 0:
            row = "| %s | — | — | — | — | 0/%d |" % (name, report["runs"])
            if kitti:
                row += " — | — |"
            if ref:
                row += " %s | — |" % (("%.3f" % ref[name]) if name in ref else "—")
            lines.append(row)
            continue
        row = "| %s | **%.4f** | %.4f | %.4f | %.4f | %d/%d |" % (
            name, a["median"], a["std"], a["min"], a["max"], n_ok, report["runs"])
        if kitti:
            t = s.get("t_rel_pct", {})
            r = s.get("r_rel_deg_per_100m", {})
            row += " %s | %s |" % (
                ("%.3f" % t["median"]) if t.get("median") is not None else "—",
                ("%.4f" % r["median"]) if r.get("median") is not None else "—")
        if ref:
            if name in ref:
                d = 100.0 * (a["median"] - ref[name]) / ref[name]
                row += " %.3f | %+.1f%% |" % (ref[name], d)
            else:
                row += " — | — |"
        lines.append(row)

    if ref:
        lines += ["",
                  "> `ref` is a published number measured on different hardware. It is "
                  "context, not a baseline: re-run the baseline config on this machine "
                  "and compare against that."]
    return "\n".join(lines) + "\n"


def compare(paths):
    reports = []
    for p in paths:
        f = os.path.join(rpath(p), "results.json")
        if not os.path.exists(f):
            sys.exit("no results.json in %s" % p)
        reports.append(json.load(open(f)))
    names = []
    for r in reports:
        for n in r["sequences"]:
            if n not in names:
                names.append(n)

    print("| seq | " + " | ".join("%s [m]" % r["tag"] for r in reports) + " | Δ vs first |")
    print("|---|" + "---|" * (len(reports) + 1))
    for n in names:
        cells, base = [], None
        for r in reports:
            a = r["sequences"].get(n, {}).get("ate_rmse", {})
            m = a.get("median")
            cells.append("—" if m is None else "%.4f" % m)
            if base is None:
                base = m
        last = reports[-1]["sequences"].get(n, {}).get("ate_rmse", {}).get("median")
        delta = "—" if (base in (None, 0) or last is None) else "%+.1f%%" % (
            100.0 * (last - base) / base)
        print("| %s | %s | %s |" % (n, " | ".join(cells), delta))
    print("\nMedians over %s runs. A change smaller than the run-to-run std is not a result."
          % "/".join(str(r["runs"]) for r in reports))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", help="config name from sequences.yaml")
    p.add_argument("--sequences", help="comma separated subset (default: all)")
    p.add_argument("--runs", type=int, default=10,
                   help="executions per sequence (default 10; the thesis used 3)")
    p.add_argument("--trajectory", default="frames", choices=["frames", "keyframes"],
                   help="score the full-frame or the keyframe trajectory (default frames)")
    p.add_argument("--tag", help="name for this result set (default: config name)")
    p.add_argument("--out", default="evaluation/results")
    p.add_argument("--timeout", type=int, default=3600, help="per-run timeout in seconds")
    p.add_argument("--registry", help="alternative sequences.yaml")
    p.add_argument("--dry-run", action="store_true",
                   help="print commands and check every input path, run nothing")
    p.add_argument("--compare", nargs="+", metavar="RESULT_DIR",
                   help="compare result directories instead of running")
    a = p.parse_args()

    if a.compare:
        compare(a.compare)
        return
    if not a.config:
        p.error("--config is required (or use --compare)")
    benchmark(a)


if __name__ == "__main__":
    main()
