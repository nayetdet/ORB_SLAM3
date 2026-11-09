#!/usr/bin/env python3
"""Run several ARMS on the same sequences, interleaved run by run, and score each run at once.

run_benchmark.py runs one config in a block (10 runs of arm A, then 10 of arm B). Anything
that drifts over hours -- thermal state, a browser, the page cache, another job -- is then
confounded with the arm. Here every arm is run once per (run, sequence) block, in an order
that is a balanced Latin square (Williams design) shuffled by --seed, so each arm occupies
each position in the block equally often and each arm is preceded by every other arm
equally often. A drift then lands on all arms alike.

    # validate inputs, print the schedule and the commands, run nothing
    python3 evaluation/harness/run_interleaved.py --arms euroc_p_base,euroc_p_c1 \\
        --sequences MH01,MH04 --runs 10 --seed 1 --out results_pilot/euroc --dry-run

    # 25 s startup check of every arm x sequence ('New Map created' must appear in the log)
    python3 evaluation/harness/run_interleaved.py --arms ... --sequences ... --out ... --preflight 25

    # the real thing; --resume continues an interrupted run
    python3 evaluation/harness/run_interleaved.py --arms euroc_p_base,euroc_p_base_b,euroc_p_c1 \\
        --sequences MH01,MH04,V103 --runs 10 --seed 1 --out results_pilot/euroc --prune-dense

    python3 evaluation/harness/run_interleaved.py --self-test

Layout (DIR = --out). Every DIR/<arm>/ is a result set that run_benchmark.py --compare and
stats_compare.py (flat layout) read unchanged:
    DIR/order.json                           plan, balance table and one record per run, for auditing
    DIR/<arm>/results.json, results.md       rewritten after every run, same format as run_benchmark.py
    DIR/<arm>/<seq>/frames_runNN.txt         frame trajectory (moved out of the run directory)
    DIR/<arm>/<seq>/runNN/slam.log, kf_runNN.txt  (+ .pcd/.ot/.bt for run00 with --prune-dense)

Beyond run_benchmark.py each run entry carries 'scores' (ATE RMSE of the frame and of the
keyframe trajectory, each SE(3) and Sim(3): the primary 'ate_rmse' stays the frame SE(3) one)
and 'log_metrics' (log_metrics.py: tracking time, dense stage times, voxel-filter calls and
overflows, final GBA, loops, re-inits ...) and, when the config has run_expect / run_forbid, 'log_check'
(did the finished run's log show the settings the arm is named for: a binary that ignores
System.syncShutdown or GlobalBA.final runs fine and says nothing, so the log is the only proof; a failed
check makes the exit status 1 and --summary lists it). results.json 'meta' records ORB_CPUSET, the CPUs the
process may use, the host load at start and the CPU governor / boost state; order.json records,
per run, the CPU time that other host processes used on those CPUs while it ran, and their mean
frequency and highest temperature.
"""

import argparse
import datetime
import glob
import hashlib
import json
import os
import random
import re
import shutil
import signal
import socket
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)

SCHEMA = 1
SCORE_KEYS = ("frames_se3", "frames_sim3", "kf_se3", "kf_sim3")
rb = ev = metrics = lm = None  # run_benchmark & friends: imported by load_harness() (numpy is not on a bare host)


def load_harness():
    global rb, ev, metrics, lm
    if rb is not None:
        return
    try:
        import run_benchmark as _rb
    except ModuleNotFoundError as exc:
        if exc.name == "numpy":
            sys.exit("numpy is missing: run inside the orb_slam3:dense image (evaluation/run_pilot.sh does) "
                     "or pip install numpy. Scheduling code needs no numpy: --self-test works anywhere.")
        raise
    import evaluate as _ev
    import log_metrics as _lm
    import metrics as _metrics
    rb, ev, metrics, lm = _rb, _ev, _metrics, _lm


# ---------------------------------------------------------------------------------------
# schedule
# ---------------------------------------------------------------------------------------

def williams_blocks(n):
    """Williams design on symbols 0..n-1, as a list of Latin blocks (each block = n rows, every symbol once per column).

    Every ordered pair of different symbols is adjacent equally often over the whole design: once for
    even n (one block), twice for odd n, whose design is the n cyclic shifts of a zigzag sequence plus
    their reversals (two blocks, 2n rows).
    """
    if n == 1:
        return [[[0]]]
    seq, lo, hi, low = [0], 1, n - 1, True
    while len(seq) < n:
        if low:
            seq.append(lo)
            lo += 1
        else:
            seq.append(hi)
            hi -= 1
        low = not low
    rows = [[(x + i) % n for x in seq] for i in range(n)]
    return [rows] if n % 2 == 0 else [rows, [list(reversed(r)) for r in rows]]


def rotation_blocks(n):
    """Plain cyclic Latin square: position balance only, carry-over is not balanced."""
    return [[[(j + i) % n for j in range(n)] for i in range(n)]]


DESIGNS = {"williams": williams_blocks, "rotation": rotation_blocks}


def make_plan(arms, sequences, runs, seed, design="williams"):
    """Interleaved schedule: a list of {index, run, seq, position, arm} in execution order.

    Order of execution: for run r, for each sequence, the arms of one design row. The arm
    labels are shuffled by the seed (on the sorted names, so the command-line order does not
    matter). Per sequence the rows are dealt to the runs in cycles: the design's Latin blocks in
    random order, the rows of each block in random order. After every n runs (n = number of arms)
    each arm has therefore occupied each position equally often in that sequence; after a whole
    cycle (n runs, or 2n for an odd n) every arm has also followed every other arm equally often.
    Other run counts leave a partial block and counts that differ by at most one. Raising --runs
    later extends the plan without changing the runs already in it.
    """
    arms = sorted(arms)
    blocks = DESIGNS[design](len(arms))
    cycle_len = sum(len(b) for b in blocks)
    label = list(range(len(arms)))
    random.Random("arms:%s" % seed).shuffle(label)
    dealt = {}  # (sequence, cycle) -> the rows of that cycle in execution order
    plan = []
    for r in range(runs):
        for seq in sequences:
            cycle, k = divmod(r, cycle_len)
            if (seq, cycle) not in dealt:
                rng = random.Random("rows:%s:%s:%d" % (seed, seq, cycle))
                order = list(blocks)
                rng.shuffle(order)
                rows = []
                for blk in order:
                    blk = list(blk)
                    rng.shuffle(blk)
                    rows += blk
                dealt[(seq, cycle)] = rows
            for pos, sym in enumerate(dealt[(seq, cycle)][k]):
                plan.append({"index": len(plan), "run": r, "seq": seq, "position": pos, "arm": arms[label[sym]]})
    return plan


def balance(plan, arms):
    """Position counts per arm (overall and per sequence), carry-over pair counts, exactness."""
    arms = sorted(arms)
    n = len(arms)
    seqs = sorted({e["seq"] for e in plan})
    pos = {a: [0] * n for a in arms}
    pos_seq = {s: {a: [0] * n for a in arms} for s in seqs}
    carry = {a: {b: 0 for b in arms} for a in arms}
    prev = {}
    for e in plan:
        pos[e["arm"]][e["position"]] += 1
        pos_seq[e["seq"]][e["arm"]][e["position"]] += 1
        key = (e["run"], e["seq"])
        if key in prev:
            carry[prev[key]][e["arm"]] += 1
        prev[key] = e["arm"]
    exact_pos = all(len(set(c)) == 1 for c in pos.values())
    exact_seq = all(len(set(c)) == 1 for s in pos_seq.values() for c in s.values())
    off = [carry[a][b] for a in arms for b in arms if a != b]
    return {"arms": arms, "positions": pos, "positions_by_sequence": pos_seq, "carryover": carry,
            "position_balanced_overall": exact_pos, "position_balanced_per_sequence": exact_seq,
            "carryover_balanced": bool(off) and len(set(off)) == 1}


def format_balance(b):
    arms = b["arms"]
    w = max(len(a) for a in arms)
    lines = ["position counts (columns = position in the block, 1 = first):"]
    for a in arms:
        lines.append("  %-*s  %s" % (w, a, " ".join("%3d" % c for c in b["positions"][a])))
    lines.append("balanced overall: %s; balanced within every sequence: %s; carry-over (who runs right "
                 "before whom) balanced: %s" % (b["position_balanced_overall"], b["position_balanced_per_sequence"],
                                                b["carryover_balanced"]))
    if not b["position_balanced_per_sequence"]:
        lines.append("  (unequal counts: --runs is not a multiple of the number of arms, so the last block of the "
                     "design is partial; the counts differ by at most one per sequence)")
    return "\n".join(lines)


# ---------------------------------------------------------------------------------------
# files, host state
# ---------------------------------------------------------------------------------------

def write_json(path, obj):
    """Atomic write: a reader (or a crash) never sees half a file."""
    tmp = "%s.tmp.%d" % (path, os.getpid())
    with open(tmp, "w") as fh:
        json.dump(obj, fh, indent=2)
    os.replace(tmp, path)


def now():
    return datetime.datetime.now().astimezone().isoformat(timespec="seconds")


def sha256(path):
    h = hashlib.sha256()
    try:
        with open(path, "rb") as fh:
            for chunk in iter(lambda: fh.read(1 << 20), b""):
                h.update(chunk)
    except OSError:
        return None
    return h.hexdigest()


def loadavg():
    try:
        return [round(x, 2) for x in os.getloadavg()]
    except OSError:
        return None


def allowed_cpus():
    try:
        return sorted(os.sched_getaffinity(0))
    except (AttributeError, OSError):
        return list(range(os.cpu_count() or 1))


def read_cpu_stat(cpus):
    """(busy_s, iowait_s) summed over the given CPUs from /proc/stat (host-wide, not namespaced)."""
    hz = os.sysconf("SC_CLK_TCK")
    want = {"cpu%d" % c for c in cpus}
    busy = iow = 0.0
    try:
        with open("/proc/stat") as fh:
            for line in fh:
                f = line.split()
                if f and f[0] in want:
                    v = [float(x) for x in f[1:]]
                    busy += v[0] + v[1] + v[2] + v[5] + v[6] + (v[7] if len(v) > 7 else 0.0)
                    iow += v[4]
    except (OSError, ValueError, IndexError):
        return None
    return busy / hz, iow / hz


def read_cgroup_cpu_s():
    """CPU seconds consumed so far by this container's cgroup (v2, then v1); None if unreadable."""
    try:
        with open("/sys/fs/cgroup/cpu.stat") as fh:
            for line in fh:
                if line.startswith("usage_usec"):
                    return int(line.split()[1]) / 1e6
    except (OSError, ValueError):
        pass
    try:
        with open("/sys/fs/cgroup/cpuacct/cpuacct.usage") as fh:
            return int(fh.read().strip()) / 1e9
    except (OSError, ValueError):
        return None


def read_mhz(cpus):
    """Mean current frequency of the given CPUs in MHz (sysfs); None if unreadable."""
    v = []
    for c in cpus:
        try:
            with open("/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq" % c) as fh:
                v.append(int(fh.read()) / 1000.0)
        except (OSError, ValueError):
            pass
    return sum(v) / len(v) if v else None


def read_temp_c():
    """Highest CPU temperature in Celsius from the k10temp / coretemp hwmon; None if unreadable."""
    best = None
    for h in glob.glob("/sys/class/hwmon/hwmon*"):
        try:
            with open(h + "/name") as fh:
                if fh.read().strip() not in ("k10temp", "coretemp", "zenpower"):
                    continue
            for t in glob.glob(h + "/temp*_input"):
                with open(t) as fh:
                    x = int(fh.read()) / 1000.0
                best = x if best is None else max(best, x)
        except (OSError, ValueError):
            pass
    return best


def cpu_policy():
    """Governor, boost switch and driver as the kernel reports them (what the earlier campaign could not prove)."""
    out = {}
    for key, path in (("governor", "/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"),
                      ("boost", "/sys/devices/system/cpu/cpufreq/boost"),
                      ("driver", "/sys/devices/system/cpu/cpu0/cpufreq/scaling_driver")):
        try:
            with open(path) as fh:
                out[key] = fh.read().strip()
        except OSError:
            out[key] = None
    return out


class HostProbe:
    """What else happened on the allowed CPUs during one run.

    other_busy_s: CPU time used there by processes outside this container -- /proc/stat busy time
    (host-wide) minus the container's own cgroup usage. An indicator of interference, not an exact
    measurement: kernel interrupts and accounting differences leave a small positive residue even
    on a quiet machine. mhz_* and temp_c_max are sampled every 2 s while the run is going.
    """

    def __init__(self, period=2.0):
        self.cpus = allowed_cpus()
        self.period = period

    def _sample(self):
        mhz, temp = read_mhz(self.cpus), read_temp_c()
        if mhz is not None:
            self.mhz.append(mhz)
        if temp is not None:
            self.temp.append(temp)

    def _loop(self):
        while not self.halt.wait(self.period):
            self._sample()

    def start(self):
        self.mhz, self.temp = [], []
        self.halt = threading.Event()
        self.t0 = (read_cpu_stat(self.cpus), read_cgroup_cpu_s(), loadavg(), time.time())
        self._sample()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.halt.set()
        self.thread.join(timeout=1.0)
        self._sample()
        s0, c0, l0, t0 = self.t0
        s1, c1 = read_cpu_stat(self.cpus), read_cgroup_cpu_s()
        out = {"cpus": self.cpus, "wall_s": round(time.time() - t0, 1), "loadavg_start": l0, "loadavg_end": loadavg(),
               "cpuset_busy_s": None, "cpuset_iowait_s": None, "container_cpu_s": None, "other_busy_s": None,
               "mhz_mean": round(sum(self.mhz) / len(self.mhz)) if self.mhz else None,
               "mhz_min": round(min(self.mhz)) if self.mhz else None,
               "temp_c_max": round(max(self.temp), 1) if self.temp else None}
        if s0 and s1:
            out["cpuset_busy_s"] = round(s1[0] - s0[0], 2)
            out["cpuset_iowait_s"] = round(s1[1] - s0[1], 2)
        if c0 is not None and c1 is not None:
            out["container_cpu_s"] = round(c1 - c0, 2)
        if out["cpuset_busy_s"] is not None and out["container_cpu_s"] is not None:
            out["other_busy_s"] = round(max(0.0, out["cpuset_busy_s"] - out["container_cpu_s"]), 2)
        return out


def wrap_argv(argv, use_stdbuf):
    """stdbuf -oL -eL keeps what a crashed or killed run printed (the C++ stream is flushed per line)."""
    if use_stdbuf and shutil.which("stdbuf"):
        return ["stdbuf", "-oL", "-eL"] + argv
    return argv


def sequence_duration_s(cfg, seq, reg):
    """Playback length of a sequence in seconds from its timestamp file; None if unavailable."""
    if "chain" in seq:
        import chain_eval
        return chain_eval.duration_s(cfg, seq, reg)
    try:
        if cfg["dataset"] == "euroc":
            path = os.path.join(rb.rpath(cfg["timestamps_dir"]), seq["times"])
            scale = 1e-9
        elif cfg["dataset"] == "kitti":
            path = os.path.join(rb.rpath(reg["roots"]["kitti"]), seq["folder"], "times.txt")
            scale = 1.0
        elif cfg["dataset"] == "tum":
            path = os.path.join(rb.rpath(cfg["associations_dir"]), seq["assoc"])
            scale = 1.0
        else:
            return None
        first = last = None
        with open(path) as fh:
            for line in fh:
                if line.strip() and not line.startswith("#"):
                    t = float(line.split()[0])
                    first = t if first is None else first
                    last = t
        return (last - first) * scale if first is not None else None
    except (OSError, ValueError, KeyError):
        return None


# ---------------------------------------------------------------------------------------
# plan / resume state
# ---------------------------------------------------------------------------------------

def run_name(r):
    return "run%02d" % r


def new_state(args, arms, seqs, plan, bal):
    return {"schema": SCHEMA, "created": now(), "updated": now(),
            "params": {"arms": sorted(arms), "sequences": list(seqs), "runs": args.runs, "seed": args.seed,
                       "design": args.design, "tag": args.tag, "timeout": args.timeout,
                       "prune_dense": bool(args.prune_dense), "stdbuf": not args.no_stdbuf,
                       "orb_cpuset": os.environ.get("ORB_CPUSET"), "argv": sys.argv},
            "balance": bal,
            "runs": [dict(e, run_name=run_name(e["run"]), status="pending", attempts=0, start=None, end=None,
                          returncode=None, seconds=None, host=None) for e in plan]}


def reconcile_state(state, args, arms, seqs, plan, bal):
    """Resume: keep what ran, append plan entries that are new (a larger --runs), refuse a different plan."""
    p = state["params"]
    if sorted(arms) != p["arms"] or list(seqs) != p["sequences"] or args.seed != p["seed"] or args.design != p["design"]:
        sys.exit("--resume: arms/sequences/seed/design differ from DIR/order.json "
                 "(arms %s, sequences %s, seed %s, design %s)" % (p["arms"], p["sequences"], p["seed"], p["design"]))
    if args.runs < p["runs"]:
        sys.exit("--resume: --runs %d is smaller than the %d the plan was made for" % (args.runs, p["runs"]))
    have = {(e["arm"], e["seq"], e["run"]): e for e in state["runs"]}
    for e in plan:
        k = (e["arm"], e["seq"], e["run"])
        if k in have:
            if have[k]["position"] != e["position"]:
                sys.exit("--resume: the schedule for %s changed; refusing to mix two plans" % (k,))
        else:
            state["runs"].append(dict(e, run_name=run_name(e["run"]), status="pending", attempts=0, start=None,
                                      end=None, returncode=None, seconds=None, host=None))
    state["runs"].sort(key=lambda e: (e["run"], e["seq"], e["position"]))
    for i, e in enumerate(state["runs"]):
        e["index"] = i
    p["runs"] = args.runs
    state["balance"] = bal
    return state


# ---------------------------------------------------------------------------------------
# one run
# ---------------------------------------------------------------------------------------

def score_run(cfg, gt_path, frames, keyframes):
    """Primary entry fields plus the extra 'scores'. Raises only if the primary evaluation fails."""
    kw = dict(est_format=cfg["est_format"], gt_format=cfg["gt_format"], max_diff=cfg.get("max_diff", 0.02))
    r = ev.evaluate(frames, gt_path, align=cfg["align"], kitti=bool(cfg.get("kitti_relative")), **kw)
    scores = {"frames_se3": None, "frames_sim3": None, "kf_se3": None, "kf_sim3": None}
    errors = []
    if cfg["align"] == "se3":
        scores["frames_se3"] = r["ate"]["rmse"]
    for key, path, align in (("frames_se3", frames, "se3"), ("frames_sim3", frames, "sim3"),
                             ("kf_se3", keyframes, "se3"), ("kf_sim3", keyframes, "sim3")):
        if scores[key] is not None or not path or not os.path.exists(path):
            continue
        try:
            scores[key] = ev.evaluate(path, gt_path, align=align, **kw)["ate"]["rmse"]
        except Exception as exc:  # a secondary score failing must not cost the run its primary one
            errors.append("%s: %s" % (key, exc))
    return r, scores, errors


def log_check(cfg, log_path):
    """The config's run_expect / run_forbid regexes (sequences.yaml) against the log of a finished run; None if it states none.

    What only shows at shutdown cannot be seen by the 25 s preflight: a binary that ignores System.syncShutdown
    or GlobalBA.final (an old build, a misspelt key) runs fine and prints nothing about it, so the run would
    pass as an arm that it is not. ok = every expected line is there and no forbidden one is."""
    expect, forbid = cfg.get("run_expect", []), cfg.get("run_forbid", [])
    if not (expect or forbid):
        return None
    try:
        with open(log_path, errors="replace") as fh:
            text = fh.read()
    except OSError:
        text = ""
    missing = [pat for pat in expect if not re.search(pat, text)]
    forbidden = [pat for pat in forbid if re.search(pat, text)]
    return {"ok": not missing and not forbidden, "missing": missing, "forbidden": forbidden}


def log_check_text(check):
    return "; ".join(["lacks %r" % p for p in check["missing"]] + ["has %r" % p for p in check["forbidden"]])


def execute(plan_entry, ctx):
    """Run one (arm, seq, run), score it, return its results.json run entry and its order.json fields."""
    arm, seq_name, r = plan_entry["arm"], plan_entry["seq"], plan_entry["run"]
    cfg, seq = ctx["cfgs"][arm], ctx["reg"]["sequences"][arm][seq_name]
    tag = run_name(r)
    argv, outputs, seq_dir = rb.build_invocation(arm, cfg, seq_name, seq, ctx["reg"], tag)
    gt_path = rb.resolve_gt(cfg, seq, seq_dir, ctx["reg"])
    seq_out = os.path.join(ctx["out"], arm, seq_name)
    work = os.path.join(seq_out, tag)
    if os.path.isdir(work) and os.listdir(work):  # a failed or interrupted earlier attempt: keep it for the audit
        k = 1
        while os.path.exists("%s.attempt%d" % (work, k)):
            k += 1
        os.rename(work, "%s.attempt%d" % (work, k))
    os.makedirs(work, exist_ok=True)
    log_path = os.path.join(work, "slam.log")

    probe = HostProbe()
    t_start = now()
    probe.start()
    rc, elapsed = rb.run_one(wrap_argv(argv, not ctx["no_stdbuf"]), work, log_path, ctx["timeout"])
    host = probe.stop()
    t_end = now()

    entry = {"run": tag, "returncode": rc, "seconds": round(elapsed, 1), "log": os.path.relpath(log_path, rb.REPO)}
    try:
        entry["log_metrics"] = lm.parse_run_dir(work)
    except Exception as exc:  # parsing is bookkeeping: never lose the run over it
        entry["log_metrics"] = {"error": str(exc)}
    if rc == 0:
        check = log_check(cfg, log_path)
        if check is not None:
            entry["log_check"] = check
    if ctx["prune_dense"] and r > 0:
        rb.prune_dense(work)  # run00 keeps its cloud for validate_dense.py; sizes are already in log_metrics

    src = os.path.join(work, outputs["frames"])
    if rc != 0 or not os.path.exists(src):
        entry["status"] = "failed"
        entry["ate_rmse"] = None
    else:
        traj = os.path.join(seq_out, "frames_%s.txt" % tag)
        shutil.move(src, traj)
        entry["trajectory"] = os.path.relpath(traj, rb.REPO)
        kf = os.path.join(work, outputs["keyframes"]) if "keyframes" in outputs else None
        try:
            if "chain" in seq:  # several sequences in one process: see chain_eval.py (validity = the maps merged)
                import chain_eval
                lmx = entry.get("log_metrics") or {}
                ch = chain_eval.score(cfg, seq, ctx["reg"], traj, gt_path, lmx.get("maps_in_atlas"), lmx.get("merges_detected"))
                entry["coverage"], entry["scores"], entry["chain"] = ch["coverage"], ch["scores"], ch["chain"]
                if ch["valid"]:
                    entry["status"], entry["ate_rmse"] = "ok", ch["ate_rmse"]
                else:  # kept for the audit under chain.*; stats_compare.py drops it and flags the status
                    entry["status"], entry["ate_rmse"] = "invalid_chain", None
                    entry["error"] = "INVALID CHAIN: " + "; ".join(ch["chain"]["invalid_reasons"])
                return entry, {"start": t_start, "end": t_end, "returncode": rc, "seconds": round(elapsed, 1),
                               "status": entry["status"], "host": host, "log_check": entry.get("log_check")}
            res, scores, errors = score_run(cfg, gt_path, traj, kf)
            entry["status"] = "ok"
            entry["ate_rmse"] = res["ate"]["rmse"]
            entry["coverage"] = res["coverage"]
            if "kitti" in res:
                entry["t_rel_pct"] = res["kitti"]["t_rel_pct"]
                entry["r_rel_deg_per_100m"] = res["kitti"]["r_rel_deg_per_100m"]
            entry["scores"] = scores
            if errors:
                entry["scores_errors"] = errors
        except Exception as exc:  # evaluation failure is a result, not a crash
            entry["status"] = "eval_error"
            entry["ate_rmse"] = None
            entry["error"] = str(exc)
    return entry, {"start": t_start, "end": t_end, "returncode": rc, "seconds": round(elapsed, 1),
                   "status": entry["status"], "host": host, "log_check": entry.get("log_check")}


def new_report(arm, cfg, ctx):
    return {"config": arm, "tag": ctx["tag_of"](arm), "runs": ctx["runs"], "trajectory": "frames",
            "align": cfg["align"], "sensor": cfg["sensor"], "started": time.strftime("%Y-%m-%d %H:%M:%S"),
            "host_cpu_count": os.cpu_count(), "sequences": {}, "meta": ctx["meta"](arm)}


def update_report(report, cfg, seq_name, entry):
    s = report["sequences"].setdefault(seq_name, {"runs": []})
    s["runs"] = [e for e in s["runs"] if e["run"] != entry["run"]] + [entry]
    s["runs"].sort(key=lambda e: e["run"])
    runs = s["runs"]
    s["ate_rmse"] = metrics.aggregate([e.get("ate_rmse") for e in runs])
    s["failures"] = sum(1 for e in runs if e["status"] != "ok")
    if cfg.get("kitti_relative"):
        s["t_rel_pct"] = metrics.aggregate([e.get("t_rel_pct") for e in runs])
        s["r_rel_deg_per_100m"] = metrics.aggregate([e.get("r_rel_deg_per_100m") for e in runs])
    keys = list(SCORE_KEYS) + sorted({k for e in runs for k in (e.get("scores") or {})} - set(SCORE_KEYS))
    s["scores"] = {k: metrics.aggregate([(e.get("scores") or {}).get(k) for e in runs]) for k in keys}


def write_report(report, out_dir):
    write_json(os.path.join(out_dir, "results.json"), report)
    md = rb.render_table(report, reg=None)
    extra = ["", "### other scores (ATE RMSE [m], median over ok runs)", "",
             "| seq | frames SE(3) | frames Sim(3) | keyframes SE(3) | keyframes Sim(3) |", "|---|---|---|---|---|"]
    for name, s in report["sequences"].items():
        cells = []
        for k in SCORE_KEYS:
            a = s.get("scores", {}).get(k, {})
            cells.append("—" if a.get("median") is None else "%.4f" % a["median"])
        extra.append("| %s | %s |" % (name, " | ".join(cells)))
    chain_keys = sorted({k for s in report["sequences"].values() for k in s.get("scores", {})} - set(SCORE_KEYS))
    if chain_keys:  # multi-sequence chains (chain_eval.py): the keys of the scores that are not the four above
        extra += ["", "### chain scores (m, median over ok runs; ok = the maps merged, see chain_eval.py)", "",
                  "| seq | " + " | ".join(chain_keys) + " | invalid |", "|---|" + "---|" * (len(chain_keys) + 1)]
        for name, s in report["sequences"].items():
            cells = []
            for k in chain_keys:
                a = s.get("scores", {}).get(k, {})
                cells.append("—" if a.get("median") is None else "%.4f" % a["median"])
            bad = sum(1 for e in s.get("runs", []) if e.get("status") == "invalid_chain")
            extra.append("| %s | %s | %d/%d |" % (name, " | ".join(cells), bad, len(s.get("runs", []))))
    with open(os.path.join(out_dir, "results.md"), "w") as fh:
        fh.write(md + "\n".join(extra) + "\n")


# ---------------------------------------------------------------------------------------
# preflight / dry run / main
# ---------------------------------------------------------------------------------------

def preflight(ctx, arms, seqs, secs):
    """Run every arm x sequence for `secs` seconds. 'New Map created' must be in the log: a timeout
    alone proves nothing (a missing TUM association file once hung silently; killed output is lost
    unless stdbuf flushed it)."""
    bad = 0
    for arm in arms:
        cfg = ctx["cfgs"][arm]
        for seq_name in seqs:
            seq = ctx["reg"]["sequences"][arm][seq_name]
            argv, _, seq_dir = rb.build_invocation(arm, cfg, seq_name, seq, ctx["reg"], "pf")
            work = os.path.join(ctx["out"], "preflight", arm, seq_name)
            shutil.rmtree(work, ignore_errors=True)
            os.makedirs(work)
            missing = [p for p in (argv[0], argv[1], argv[2], seq_dir) if not os.path.exists(p)]
            log_path = os.path.join(work, "slam.log")
            if missing:
                print("  %-22s %-5s FAIL missing input %s" % (arm, seq_name, missing[0]))
                bad += 1
                continue
            rc, el = rb.run_one(wrap_argv(argv, not ctx["no_stdbuf"]), work, log_path, secs)
            with open(log_path, errors="replace") as fh:
                text = fh.read()
            why = []
            if "New Map created" not in text:
                why.append("no 'New Map created' in the log")
            if rc not in (0, -9):
                why.append("exit code %d (-9 is the harness timeout, i.e. alive)" % rc)
            for pat in cfg.get("preflight_expect", []):
                if not re.search(pat, text):
                    why.append("log lacks %r" % pat)
            for pat in cfg.get("preflight_forbid", []):
                if re.search(pat, text):
                    why.append("log has %r" % pat)
            print("  %-22s %-5s %s  rc=%d %.0fs %s" % (arm, seq_name, "ok  " if not why else "FAIL", rc, el, "; ".join(why)))
            if why:
                bad += 1
                tail = [ln for ln in text.splitlines() if ln.strip()][-3:]
                for ln in tail:
                    print("        | " + ln[:150])
    print("preflight: %s" % ("all %d pairs started" % (len(arms) * len(seqs)) if not bad else "%d pair(s) FAILED" % bad))
    return 1 if bad else 0


def dry_run(args, reg, cfgs, arms, seqs, plan, bal):
    missing = []
    first = {a: None for a in arms}
    for arm in arms:
        cfg = cfgs[arm]
        for seq_name in seqs:
            seq = reg["sequences"][arm][seq_name]
            argv, outputs, seq_dir = rb.build_invocation(arm, cfg, seq_name, seq, reg, run_name(0))
            gt_path = rb.resolve_gt(cfg, seq, seq_dir, reg)
            for label, p in (("binary", argv[0]), ("vocabulary", argv[1]), ("settings", argv[2]),
                             ("sequence", seq_dir), ("ground truth", gt_path)):
                if not os.path.exists(p):
                    missing.append("%s/%s: %s not found -> %s" % (arm, seq_name, label, p))
            if "chain" in seq:
                import chain_eval
                missing += ["%s/%s: %s" % (arm, seq_name, m) for m in chain_eval.missing_inputs(cfg, seq, reg)]
            if first[arm] is None:
                first[arm] = argv
                print("[%s %s] %s" % (arm, seq_name, " ".join(wrap_argv(argv, not args.no_stdbuf))))
                print("        gt: %s" % gt_path)
    print()
    print(format_balance(bal))
    print()
    print("first blocks of the schedule (run, sequence: arms in execution order):")
    shown = 0
    i = 0
    while i < len(plan) and shown < 6:
        j = i
        while j < len(plan) and (plan[j]["run"], plan[j]["seq"]) == (plan[i]["run"], plan[i]["seq"]):
            j += 1
        print("  %s %-5s %s" % (run_name(plan[i]["run"]), plan[i]["seq"], " > ".join(e["arm"] for e in plan[i:j])))
        shown += 1
        i = j
    total = 0.0
    unknown = 0
    for e in plan:
        d = sequence_duration_s(cfgs[e["arm"]], reg["sequences"][e["arm"]][e["seq"]], reg)
        if d is None:
            unknown += 1
        else:
            total += d + args.overhead_s
    print()
    print("%d runs in total (%d arms x %d sequences x %d runs); estimated wall time at real-time playback "
          "+ %.0f s fixed overhead per run: %s%s (dense arms add queue drain / final-pose work; a "
          "CPU limit can add more)" % (len(plan), len(arms), len(seqs), args.runs, args.overhead_s,
                                       "%.1f h" % (total / 3600.0) if total >= 3600 else "%.0f min" % (total / 60.0),
                                       "" if not unknown else " (%d runs without a known duration)" % unknown))
    if missing:
        print("\nMissing inputs:")
        for m in missing:
            print("  " + m)
    print("\ndry run: nothing executed. %d arm(s) x %d sequence(s) checked, %d missing input(s)."
          % (len(arms), len(seqs), len(missing)))
    return 1 if missing else 0


def summarize(out, interference_s=5.0):
    """Audit view of DIR/order.json (no numpy needed): what ran, what failed, who else used the CPUs."""
    path = os.path.join(out, "order.json")
    with open(path) as fh:
        st = json.load(fh)
    runs = st["runs"]
    p = st["params"]
    print("%s: created %s, updated %s%s" % (path, st["created"], st["updated"],
                                           (", finished " + st["finished"]) if st.get("finished") else ", NOT finished"))
    print("plan: %d arms x %d sequences x %d runs = %d runs (seed %s, design %s, ORB_CPUSET=%s)"
          % (len(p["arms"]), len(p["sequences"]), p["runs"], len(runs), p["seed"], p["design"], p.get("orb_cpuset")))
    by = {}
    for e in runs:
        by[e["status"]] = by.get(e["status"], 0) + 1
    print("status: " + ", ".join("%s %d" % kv for kv in sorted(by.items())))

    def med(v):
        v = sorted(x for x in v if x is not None)
        return None if not v else (v[len(v) // 2] if len(v) % 2 else 0.5 * (v[len(v) // 2 - 1] + v[len(v) // 2]))

    def f(x, spec="%.1f"):
        return "-" if x is None else spec % x

    print("\n%-22s %6s %6s %9s %10s %10s %9s" % ("arm", "ok", "failed", "median s", "other cpu", "max other", "MHz"))
    for arm in p["arms"]:
        mine = [e for e in runs if e["arm"] == arm]
        oth = [(e.get("host") or {}).get("other_busy_s") for e in mine]
        print("%-22s %6d %6d %9s %10s %10s %9s" % (
            arm, sum(1 for e in mine if e["status"] == "ok"), sum(1 for e in mine if e["status"] in ("failed", "eval_error")),
            f(med([e["seconds"] for e in mine])), f(med(oth)), f(max([x for x in oth if x is not None], default=None)),
            f(med([(e.get("host") or {}).get("mhz_mean") for e in mine]), "%.0f")))
    bad = [e for e in runs if e["status"] not in ("ok", "pending")]
    for e in bad:
        print("  %s: %s %s %s rc=%s" % (e["status"].upper(), e["arm"], e["seq"], e["run_name"], e["returncode"]))
    checked = [e for e in runs if e.get("log_check")]
    unchecked = [e for e in checked if not e["log_check"]["ok"]]
    print("\nlog check (run_expect / run_forbid of the arm's config): %d of %d checked runs failed" % (len(unchecked), len(checked)))
    for e in unchecked[:10]:
        print("  %s %s %s: %s" % (e["arm"], e["seq"], e["run_name"], log_check_text(e["log_check"])))
    noisy = sorted(((e["host"]["other_busy_s"], e) for e in runs if (e.get("host") or {}).get("other_busy_s") is not None
                    and e["host"]["other_busy_s"] >= interference_s), key=lambda t: -t[0])
    print("\nruns with >= %.0f s of CPU used on the allowed CPUs by other host processes: %d of %d"
          % (interference_s, len(noisy), sum(1 for e in runs if e.get("host"))))
    for v, e in noisy[:10]:
        print("  %6.1f s  %s %s %s (load %s)" % (v, e["arm"], e["seq"], e["run_name"], (e["host"].get("loadavg_start") or ["-"])[0]))
    temps = [(e.get("host") or {}).get("temp_c_max") for e in runs]
    temps = [t for t in temps if t is not None]
    if temps:
        print("highest CPU temperature during a run: %.1f C" % max(temps))
    starts = [e["start"] for e in runs if e.get("start")]
    ends = [e["end"] for e in runs if e.get("end")]
    if starts and ends:
        t0 = datetime.datetime.fromisoformat(min(starts))
        t1 = datetime.datetime.fromisoformat(max(ends))
        print("first run started %s, last run ended %s (%.2f h)" % (min(starts), max(ends), (t1 - t0).total_seconds() / 3600.0))
    return 0


def build_parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--arms", help="comma separated config names from sequences.yaml (each is one arm)")
    p.add_argument("--sequences", help="comma separated sequence names, common to all arms")
    p.add_argument("--runs", type=int, default=10, help="runs per arm and sequence (default 10)")
    p.add_argument("--seed", type=int, default=1, help="seed of the interleaving schedule (default 1)")
    p.add_argument("--out", help="result directory DIR (DIR/<arm>/results.json, DIR/order.json)")
    p.add_argument("--tag", default="", help="prefix of the 'tag' written in every results.json (tag = PREFIX_<arm>)")
    p.add_argument("--design", choices=sorted(DESIGNS), default="williams",
                   help="williams: balanced for position and carry-over (default); rotation: cyclic Latin square")
    p.add_argument("--timeout", type=int, default=3600, help="per-run timeout in seconds")
    p.add_argument("--registry", help="alternative sequences.yaml")
    p.add_argument("--resume", action="store_true", help="skip (arm, seq, run) already in DIR/order.json with returncode 0")
    p.add_argument("--prune-dense", action="store_true",
                   help="delete .pcd/.ot/.bt after every run except run00 (log_metrics has read them by then)")
    p.add_argument("--min-free-gb", type=float, default=3.0,
                   help="stop cleanly (resumable) when the disk of --out has less free space (default 3)")
    p.add_argument("--preflight", type=int, metavar="SECONDS",
                   help="run every arm x sequence for SECONDS and require 'New Map created'; then exit")
    p.add_argument("--no-stdbuf", action="store_true", help="do not wrap the SLAM command in stdbuf -oL -eL")
    p.add_argument("--overhead-s", type=float, default=20.0, help="fixed per-run overhead used by the --dry-run estimate")
    p.add_argument("--dry-run", action="store_true", help="check every input, print the schedule, run nothing")
    p.add_argument("--summary", action="store_true", help="print an audit summary of DIR/order.json (--out) and exit; no numpy")
    p.add_argument("--self-test", action="store_true", help="test the scheduling and bookkeeping code and exit")
    return p


class Stop(Exception):
    pass


def _terminate(signum, frame):
    raise Stop("signal %d" % signum)


def main():
    args = build_parser().parse_args()
    if args.self_test:
        sys.exit(run_self_test())
    if args.summary:
        if not args.out:
            build_parser().error("--summary needs --out DIR")
        out = os.path.expanduser(args.out)
        sys.exit(summarize(out if os.path.isabs(out) else os.path.join(REPO, out)))
    if not (args.arms and args.sequences and args.out):
        build_parser().error("--arms, --sequences and --out are required")
    load_harness()
    reg = rb.load_registry(args.registry)
    arms = [a for a in args.arms.split(",") if a]
    seqs = [s for s in args.sequences.split(",") if s]
    if len(set(arms)) != len(arms) or len(set(seqs)) != len(seqs):
        sys.exit("--arms and --sequences must not repeat a name")
    for a in arms:
        if a not in reg["configs"]:
            sys.exit("unknown arm %r (known: %s)" % (a, ", ".join(reg["configs"])))
        for s in seqs:
            if s not in reg["sequences"].get(a, {}):
                sys.exit("unknown sequence %r for %s (known: %s)" % (s, a, ", ".join(reg["sequences"].get(a, {}))))
    cfgs = {a: reg["configs"][a] for a in arms}
    if len({c["dataset"] for c in cfgs.values()}) != 1:
        sys.exit("all arms must use the same dataset, got %s" % sorted({c["dataset"] for c in cfgs.values()}))
    dataset = next(iter(cfgs.values()))["dataset"]

    plan = make_plan(arms, seqs, args.runs, args.seed, args.design)
    bal = balance(plan, arms)
    if args.dry_run:
        sys.exit(dry_run(args, reg, cfgs, arms, seqs, plan, bal))

    out = rb.rpath(args.out)
    os.makedirs(out, exist_ok=True)

    def tag_of(arm):
        if not args.tag:
            return arm
        return args.tag + ("" if args.tag[-1] in "_-." else "_") + arm

    cpuset = os.environ.get("ORB_CPUSET")
    started = now()
    start_load = loadavg()
    start_affinity = allowed_cpus()

    def meta(arm):
        cfg = cfgs[arm]
        settings = {}
        for s in seqs:
            sp = rb.rpath(reg["sequences"][arm][s].get("settings", cfg.get("settings")))
            settings[os.path.relpath(sp, rb.REPO)] = sha256(sp)
        return {"tool": "run_interleaved.py", "schema": SCHEMA, "arm": arm, "dataset": dataset,
                "arms": sorted(arms), "sequences": seqs, "runs": args.runs, "seed": args.seed, "design": args.design,
                "order_file": "../order.json", "started": started, "hostname": socket.gethostname(),
                "orb_cpuset": cpuset, "affinity_cpus": start_affinity, "affinity_count": len(start_affinity),
                "host_cpu_count": os.cpu_count(), "loadavg_start": start_load, "cpu_policy": cpu_policy(),
                "temp_c_start": read_temp_c(),
                "binary": {"path": os.path.relpath(rb.rpath(cfg["binary"]), rb.REPO), "sha256": sha256(rb.rpath(cfg["binary"]))},
                "settings_sha256": settings, "stdbuf": (not args.no_stdbuf) and bool(shutil.which("stdbuf")),
                "python": sys.version.split()[0], "argv": sys.argv}

    ctx = {"reg": reg, "cfgs": cfgs, "out": out, "timeout": args.timeout, "prune_dense": args.prune_dense,
           "no_stdbuf": args.no_stdbuf, "runs": args.runs, "tag_of": tag_of, "meta": meta}

    if args.preflight:
        print("preflight: %d arms x %d sequences, %d s each, CPUs %s (ORB_CPUSET=%s)"
              % (len(arms), len(seqs), args.preflight, start_affinity, cpuset))
        sys.exit(preflight(ctx, arms, seqs, args.preflight))

    order_path = os.path.join(out, "order.json")
    if os.path.exists(order_path):
        if not args.resume:
            sys.exit("%s exists: use --resume to continue it, or choose another --out" % order_path)
        with open(order_path) as fh:
            state = reconcile_state(json.load(fh), args, arms, seqs, plan, bal)
    else:
        state = new_state(args, arms, seqs, plan, bal)

    reports = {}
    for arm in arms:
        rpath_ = os.path.join(out, arm, "results.json")
        if args.resume and os.path.exists(rpath_):
            with open(rpath_) as fh:
                reports[arm] = json.load(fh)
            old_meta = reports[arm].get("meta", {})
            reports[arm]["runs"] = args.runs
            reports[arm]["meta"] = meta(arm)
            reports[arm]["meta"]["started"] = old_meta.get("started", started)
            reports[arm]["meta"]["resumed"] = old_meta.get("resumed", []) + [started]
        else:
            reports[arm] = new_report(arm, cfgs[arm], ctx)
        os.makedirs(os.path.join(out, arm), exist_ok=True)

    def have_result(e):
        s = reports[e["arm"]]["sequences"].get(e["seq"], {})
        return any(x["run"] == e["run_name"] for x in s.get("runs", []))

    # an entry is done when it ran with returncode 0 AND its result is in results.json (otherwise re-run it)
    todo = [e for e in state["runs"] if not (e["returncode"] == 0 and have_result(e))]
    print("interleaved run: %d arms x %d sequences x %d runs = %d runs; %d to do; CPUs %s (ORB_CPUSET=%s); load %s"
          % (len(arms), len(seqs), args.runs, len(state["runs"]), len(todo), start_affinity, cpuset, start_load))
    print(format_balance(bal))
    write_json(order_path, state)
    for arm in arms:
        write_report(reports[arm], os.path.join(out, arm))

    signal.signal(signal.SIGTERM, _terminate)
    done = 0
    try:
        for e in todo:
            free_gb = shutil.disk_usage(out).free / 1e9
            if free_gb < args.min_free_gb:
                print("\nSTOP: only %.1f GB free under %s (< --min-free-gb %.1f). Free space, then --resume."
                      % (free_gb, out, args.min_free_gb))
                write_json(order_path, state)
                sys.exit(3)
            arm, seq_name = e["arm"], e["seq"]
            e["attempts"] += 1
            e["status"], e["start"], e["end"], e["returncode"] = "running", now(), None, None
            state["updated"] = now()
            write_json(order_path, state)
            print("  [%d/%d] %s %s %s %-20s ... " % (e["index"] + 1, len(state["runs"]), seq_name, e["run_name"],
                                                    "pos%d" % (e["position"] + 1), arm), end="", flush=True)
            entry, rec = execute(e, ctx)
            e.update(rec)
            update_report(reports[arm], cfgs[arm], seq_name, entry)
            write_report(reports[arm], os.path.join(out, arm))
            write_json(order_path, state)
            done += 1
            if entry["status"] == "ok":
                msg = "ATE %.4f m (cov %.0f%%, %.0fs)" % (entry["ate_rmse"], 100 * entry["coverage"], entry["seconds"])
                if "t_rel_pct" in entry:
                    msg += "  t_rel %.2f%%" % entry["t_rel_pct"]
                oth = rec["host"]["other_busy_s"]
                if oth is not None:
                    msg += "  other-cpu %.1fs" % oth
                if rec["log_check"] and not rec["log_check"]["ok"]:
                    msg += "  LOG CHECK FAILED: " + log_check_text(rec["log_check"])
                print(msg)
            else:
                print("%s (rc=%s, %.0fs) -- see %s%s" % (entry["status"].upper(), entry["returncode"], entry["seconds"],
                                                         entry["log"], ("  [" + entry["error"] + "]") if entry.get("error") else ""))
    except (KeyboardInterrupt, Stop) as exc:
        print("\ninterrupted (%s): order.json and results.json are current up to the last finished run; "
              "re-run with --resume" % (str(exc) or "Ctrl-C"))
        write_json(order_path, state)
        sys.exit(130)
    state["updated"] = now()
    state["finished"] = now()
    write_json(order_path, state)
    bad = sum(1 for e in state["runs"] if e["status"] != "ok")
    unchecked = [e for e in state["runs"] if (e.get("log_check") or {}).get("ok") is False]
    print("\ndone: %d run(s) executed now; %d of %d runs not ok; written under %s" % (done, bad, len(state["runs"]), out))
    if unchecked:
        print("LOG CHECK FAILED in %d run(s): the log lacks a line the arm's settings imply, or has one they exclude, so the "
              "arm may not have run with the settings it is named for (binary or settings file out of date?); "
              "see run_interleaved.py --summary --out %s" % (len(unchecked), args.out))
    sys.exit(1 if bad or unchecked else 0)


# ---------------------------------------------------------------------------------------
# self-test (no numpy, no dataset)
# ---------------------------------------------------------------------------------------

def run_self_test():
    import tempfile
    failures = []

    def check(name, cond, info=""):
        print(("PASS " if cond else "FAIL ") + name + (("  " + info) if info else ""))
        if not cond:
            failures.append(name)

    for n in (1, 2, 3, 4, 5, 6, 7):
        blocks = williams_blocks(n)
        rows = [r for blk in blocks for r in blk]
        ok_blocks = all(sorted(r[j] for r in blk) == list(range(n)) for blk in blocks for j in range(n))
        pairs = {}
        for r in rows:
            for x, y in zip(r, r[1:]):
                pairs[(x, y)] = pairs.get((x, y), 0) + 1
        ok_pairs = n == 1 or (len(pairs) == n * (n - 1) and len(set(pairs.values())) == 1)
        check("Williams design n=%d: %d block(s), %d rows, each block a Latin square, carry-over balanced over the design"
              % (n, len(blocks), len(rows)), ok_blocks and ok_pairs)
    check("rotation design is a Latin square", all(sorted(r[j] for r in rotation_blocks(5)[0]) == list(range(5)) for j in range(5)))

    arms = ["a", "b", "c", "d", "e"]
    plan = make_plan(arms, ["S1", "S2", "S3"], 10, 7)
    b = balance(plan, arms)
    check("5 arms x 3 sequences x 10 runs: 150 entries, exactly position- and carry-over-balanced in every sequence",
          len(plan) == 150 and b["position_balanced_overall"] and b["position_balanced_per_sequence"] and b["carryover_balanced"])
    check("each arm exactly once per (run, sequence) block",
          all(sorted(e["arm"] for e in plan if (e["run"], e["seq"]) == (r, s)) == arms for r in range(10) for s in ("S1", "S2", "S3")))
    check("execution order is run-major, then sequence, then position",
          [(e["run"], e["seq"], e["position"]) for e in plan] == sorted((e["run"], e["seq"], e["position"]) for e in plan)
          and [e["index"] for e in plan] == list(range(150)))
    check("deterministic for a seed, independent of the arm order given", plan == make_plan(list(reversed(arms)), ["S1", "S2", "S3"], 10, 7))
    check("another seed gives another schedule", plan != make_plan(arms, ["S1", "S2", "S3"], 10, 8))
    check("raising --runs keeps the runs already planned", make_plan(arms, ["S1", "S2", "S3"], 14, 7)[:150] == plan)
    for runs in (5, 10, 15):
        pb = balance(make_plan(arms, ["S1", "S2"], runs, 4), arms)
        check("5 arms, %d runs: position-balanced per sequence%s" % (runs, ", carry-over balanced" if runs % 10 == 0 else ""),
              pb["position_balanced_per_sequence"] and (runs % 10 != 0 or pb["carryover_balanced"]))
    p4 = make_plan(["w", "x", "y", "z"], ["S"], 8, 3)
    check("4 arms x 8 runs (2 full cycles): balanced", balance(p4, ["w", "x", "y", "z"])["position_balanced_per_sequence"])
    p42 = make_plan(["w", "x", "y", "z"], ["S"], 6, 3)  # 6 runs = one Latin square plus half of the next
    b42 = balance(p42, ["w", "x", "y", "z"])
    spread = max(max(c) - min(c) for c in b42["positions"].values())
    check("a partial block (6 runs of 4 arms) is reported as unbalanced and is within 1", not b42["position_balanced_per_sequence"] and spread <= 1,
          "spread %d" % spread)
    check("single arm: trivial plan", [e["arm"] for e in make_plan(["only"], ["S"], 3, 1)] == ["only"] * 3)
    check("rotation design plan is position-balanced", balance(make_plan(arms, ["S"], 5, 1, "rotation"), arms)["position_balanced_per_sequence"])

    class A:  # stand-in for the parsed arguments
        runs, seed, design, tag, timeout, prune_dense, no_stdbuf = 4, 7, "williams", "", 100, False, False
    small = make_plan(["a", "b"], ["S"], 4, 7)
    st = new_state(A, ["a", "b"], ["S"], small, balance(small, ["a", "b"]))
    check("new order.json state lists every planned run as pending", len(st["runs"]) == 8 and all(e["status"] == "pending" for e in st["runs"]))
    st["runs"][0]["returncode"] = 0
    st["runs"][0]["status"] = "ok"
    A.runs = 6
    bigger = make_plan(["a", "b"], ["S"], 6, 7)
    st = reconcile_state(st, A, ["a", "b"], ["S"], bigger, balance(bigger, ["a", "b"]))
    check("resume with a larger --runs appends and keeps finished entries",
          len(st["runs"]) == 12 and st["runs"][0]["returncode"] == 0 and [e["index"] for e in st["runs"]] == list(range(12)))

    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "x.json")
        write_json(path, {"a": 1})
        write_json(path, {"a": 2})
        check("atomic JSON write leaves no temp file", json.load(open(path)) == {"a": 2} and os.listdir(d) == ["x.json"])
    cpus = allowed_cpus()
    s = read_cpu_stat(cpus)
    check("host CPU counters readable on the allowed CPUs", s is None or (s[0] >= 0 and s[1] >= 0), "cpus %s -> %s" % (cpus, s))
    pr = HostProbe()
    pr.start()
    time.sleep(0.05)
    r = pr.stop()
    check("HostProbe returns the documented fields", set(r) >= {"cpus", "wall_s", "loadavg_start", "cpuset_busy_s", "container_cpu_s",
                                                                 "other_busy_s", "mhz_mean", "mhz_min", "temp_c_max"})
    check("CPU policy and sensors are read without raising", set(cpu_policy()) == {"governor", "boost", "driver"}
          and (read_mhz(cpus) is None or read_mhz(cpus) > 0))
    with tempfile.TemporaryDirectory() as d:
        fake = {"schema": 1, "created": "2026-01-01T00:00:00+00:00", "updated": "2026-01-01T01:00:00+00:00",
                "params": {"arms": ["a", "b"], "sequences": ["S"], "runs": 2, "seed": 1, "design": "williams", "orb_cpuset": "0,1"},
                "runs": [{"arm": "a", "seq": "S", "run_name": "run00", "status": "ok", "seconds": 10.0, "returncode": 0,
                          "start": "2026-01-01T00:00:00+00:00", "end": "2026-01-01T00:10:00+00:00",
                          "host": {"other_busy_s": 7.0, "mhz_mean": 3500.0, "temp_c_max": 60.0, "loadavg_start": [1.0]}},
                         {"arm": "b", "seq": "S", "run_name": "run00", "status": "failed", "seconds": 5.0, "returncode": -9,
                          "start": "2026-01-01T00:10:00+00:00", "end": "2026-01-01T00:20:00+00:00", "host": None},
                         {"arm": "a", "seq": "S", "run_name": "run01", "status": "pending", "seconds": None, "returncode": None,
                          "start": None, "end": None, "host": None}]}
        write_json(os.path.join(d, "order.json"), fake)
        import contextlib
        import io
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            rc = summarize(d)
        out = buf.getvalue()
        check("--summary prints status, failures, interference and the span", rc == 0 and "ok 1" in out and "FAILED: b S run00" in out
              and "7.0 s  a S run00" in out and "NOT finished" in out and "0.33 h" in out)
    with tempfile.TemporaryDirectory() as d:
        lp = os.path.join(d, "slam.log")
        with open(lp, "w") as fh:
            fh.write("Shutdown\nShutdown: waited 0.01 s for LocalMapping/LoopClosing/GBA\nFinal global bundle adjustment done (1 map(s))\n")
        both = {"run_expect": ["Shutdown: waited", "Final global bundle adjustment done"], "run_forbid": ["Dense Reconstruction timing"]}
        c1 = log_check(both, lp)
        c2 = log_check({"run_expect": ["Shutdown: waited", "Dense Reconstruction timing"], "run_forbid": ["Final global bundle adjustment"]}, lp)
        check("log check: expected lines present and forbidden ones absent -> ok", c1 == {"ok": True, "missing": [], "forbidden": []})
        check("log check: a missing expected line and a present forbidden line are both named",
              c2["ok"] is False and c2["missing"] == ["Dense Reconstruction timing"] and c2["forbidden"] == ["Final global bundle adjustment"]
              and "lacks 'Dense Reconstruction timing'" in log_check_text(c2))
        check("log check: a config without run_expect / run_forbid is not checked", log_check({"align": "se3"}, lp) is None)
        check("log check: an unreadable log fails an expectation instead of raising", log_check(both, os.path.join(d, "none.log"))["ok"] is False)
        fake["runs"][0]["log_check"] = {"ok": False, "missing": ["Shutdown: waited"], "forbidden": []}
        write_json(os.path.join(d, "order.json"), fake)
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            summarize(d)
        check("--summary lists the runs that failed the log check", "1 of 1 checked runs failed" in buf.getvalue()
              and "lacks 'Shutdown: waited'" in buf.getvalue())
    check("stdbuf wrapper", wrap_argv(["x"], False) == ["x"] and wrap_argv(["x"], True)[-1] == "x")
    print("\nself-test: %s" % ("ALL PASSED" if not failures else "FAILED: " + ", ".join(failures)))
    return 0 if not failures else 1


if __name__ == "__main__":
    main()
