#!/usr/bin/env python3
"""Arm-vs-baseline statistics for the ORB-SLAM3 benchmark (dense reconstruction study).

Dependency-free: Python >= 3.8, standard library only (no numpy / scipy).

    python3 evaluation/harness/stats_compare.py --results DIR --out DIR [--jobs N]
    python3 evaluation/harness/stats_compare.py --self-test

    # flat layout of run_interleaved.py: DIR/<arm>/results.json, arm names are free
    python3 evaluation/harness/stats_compare.py --results DIR --out OUT --baseline euroc_p_base \\
        --arms euroc_p_base_b,euroc_p_c1,euroc_p_faithful,euroc_p_imp --null-arms euroc_p_base_b \\
        --pair euroc_p_c1:euroc_p_imp

INPUT   DIR/<dataset>_<arm>/results.json as written by run_benchmark.py
        (dataset: tum | euroc | kitti;  arm: baseline | dense_faithful | dense_imp | offline).
        Per sequence: runs[] with run, status, returncode, seconds, ate_rmse [m], coverage and,
        for KITTI, t_rel_pct. Nothing under DIR is modified.
        Flat layout (--layout flat, chosen automatically when DIR/<baseline>/results.json exists):
        DIR/<arm>/results.json with any arm name, all of one dataset (results.json meta.dataset,
        else the first word of its config name). --baseline names the reference arm, --arms the
        arms compared with it (default: every other arm found).
OUTPUT  OUT/stats_all.json   every number (groups, comparisons, summaries a-d, flags)
        OUT/stats_table.md   the same as tables

DEFINITIONS (project-wide, so that independent implementations agree)
  Sample     per (dataset, sequence, arm): the metric of every run with status == "ok" and a finite
             value. Failed runs are dropped (n_ok is reported); they are not imputed.
  Test       for each dataset, sequence and non-baseline arm: arm vs the same dataset's baseline.
             statistic = |median(arm) - median(baseline)|   (median of an even n = mean of the two
             middle values). Two-sided permutation p-value over relabelings of the pooled values:
             exact enumeration of all C(n1+n2, n1) relabelings when that is <= 400000, otherwise
             Monte Carlo with 200000 shuffles (random.Random(12345)).
             p = (number of relabelings with statistic >= observed - 1e-12) / total.
  Holm       Holm-Bonferroni step-down (cumulative max, capped at 1) over the whole family of ATE
             comparisons (all sequence x arm) -> p_holm_family, and within each dataset ->
             p_holm_dataset. KITTI t_rel_pct is a separate family (Holm over its own comparisons).
  Ratio      median(arm) / median(baseline) with a 95% percentile bootstrap CI: 10000 resamples,
             each arm resampled independently with replacement, ratio of the resampled medians,
             percentiles 2.5 / 97.5 by linear interpolation. random.Random(12345) is re-seeded for
             every comparison, so a result does not depend on the order or the number of workers.
  better / worse  = lower / higher median than baseline (every metric here is an error).
  Summaries  (a) counts of raw p<0.05 and Holm p<0.05 per arm and overall, vs the number expected
             by chance; (b) the null-control arm (default "offline"): how many of its comparisons
             reach raw p<0.05; (c) per arm, sequences better / worse at raw p<0.05 and Holm p<0.05
             and where the ratio CI upper bound exceeds 1.10; (d) wins / losses / ties by dataset.
  Sensitivity (extra, not part of the project-wide definition): Holm without the null-control arm;
             expected number of raw hits at the null arm's empirical rate; p with ties counted as not
             extreme; pooled run-order diagnostics (trend and lag-1 autocorrelation inside blocks); the
             smallest p of the exact test for equal sample sizes and the raw-p bound Holm needs at rank 1.
  Flags      failed runs, n_ok below the declared run count, outliers (Tukey 1.5 / 3 x IQR), exact
             ties, tie mass of the permutation distribution at the observed value, heavy spread,
             coverage differences (the ATE is then computed on different numbers of poses),
             wall-time outliers, disagreement with the aggregates stored in results.json.
  Null arms  --null-arms a,b (default: --null-arm, "offline"; none in the flat layout) names arms
             that should not differ from the baseline: "offline" by construction in the legacy
             study, an A/A replicate (same settings, other name) in an interleaved pilot. Their
             comparisons are tested like any other (and stay in the Holm families, with a
             sensitivity column without them); section (b) reports how many reach raw p<alpha
             as the empirical false-positive rate, with its Clopper-Pearson interval.
  Pairs      --pair REF:ARM (repeatable, or a comma list) adds comparisons of ARM against REF
             instead of the baseline, e.g. a baseline+C1 arm against dense_imp to separate the
             dense effect from the final BA. Same test, ratio and bootstrap; they form their own
             Holm family (one per metric, over all pairs x sequences), are listed in their own
             section and never enter the arm-vs-baseline families or summaries (a)-(d).
  Metrics    --metrics ate_rmse,t_rel_pct (default). Any other per-run number can be added by its
             dotted path in the run entry (scores.kf_se3, scores.frames_sim3, seconds,
             log_metrics.tracking_time_mean_s, ...). Every metric is a cost (lower = better) and is
             its own Holm family; do not add one where more is not worse (point counts, file sizes).
"""

import argparse
import datetime
import hashlib
import itertools
import json
import math
import os
import platform
import random
import sys
import time

SCHEMA_VERSION = 1
TOL = 1e-12
DATASET_ORDER = ["tum", "euroc", "kitti"]
ARM_ORDER = ["dense_faithful", "dense_imp", "offline"]
METRIC_LABEL = {"ate_rmse": "ATE RMSE [m]", "t_rel_pct": "KITTI t_rel [%]"}
METRICS = ["ate_rmse", "t_rel_pct"]


def mlabel(metric):
    return METRIC_LABEL.get(metric, metric)


# --------------------------------------------------------------------------- basic statistics

def median_sorted(v):
    n = len(v)
    h = n // 2
    return v[h] if n % 2 else (v[h - 1] + v[h]) / 2.0


def median(values):
    return median_sorted(sorted(values))


def mean(values):
    return math.fsum(values) / len(values)


def stdev(values):
    """Sample standard deviation (ddof=1); None for n < 2."""
    n = len(values)
    if n < 2:
        return None
    m = mean(values)
    return math.sqrt(math.fsum((x - m) ** 2 for x in values) / (n - 1))


def quantile_linear(sorted_vals, q):
    """Quantile by linear interpolation between order statistics (numpy's default method)."""
    n = len(sorted_vals)
    pos = (n - 1) * q
    lo = int(math.floor(pos))
    hi = min(lo + 1, n - 1)
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (pos - lo)


# --------------------------------------------------------------------------- permutation test

def _exact_counts(v, n1, observed, tol):
    """Enumerate every size-n1 subset of the sorted pooled list v (the 'arm' labeling).

    Returns (#relabelings with stat >= observed - tol, #relabelings with |stat - observed| <= tol).
    """
    n = len(v)
    n2 = n - n1
    a_lo, a_hi = (n1 - 1) // 2, n1 // 2
    b_lo, b_hi = (n2 - 1) // 2, n2 // 2
    thr, hi = observed - tol, observed + tol
    rng = range(n)
    ge = tie = 0
    for c in itertools.combinations(rng, n1):
        s = set(c)
        b = [v[i] for i in rng if i not in s]          # complement, sorted because v is sorted
        d = abs(0.5 * (v[c[a_lo]] + v[c[a_hi]]) - 0.5 * (b[b_lo] + b[b_hi]))
        if d >= thr:
            ge += 1
            if d <= hi:
                tie += 1
    return ge, tie


def _mc_counts(arm, base, observed, tol, shuffles, seed):
    rng = random.Random(seed)
    pool = list(arm) + list(base)
    n1 = len(arm)
    thr, hi = observed - tol, observed + tol
    ge = tie = 0
    shuffle = rng.shuffle
    for _ in range(shuffles):
        shuffle(pool)
        d = abs(median_sorted(sorted(pool[:n1])) - median_sorted(sorted(pool[n1:])))
        if d >= thr:
            ge += 1
            if d <= hi:
                tie += 1
    return ge, tie


def permutation_test(arm, base, exact_max=400000, mc_shuffles=200000, seed=12345, tol=TOL):
    n1, n2 = len(arm), len(base)
    observed = abs(median(arm) - median(base))
    total_exact = math.comb(n1 + n2, n1)
    if total_exact <= exact_max:
        method, total = "exact", total_exact
        ge, tie = _exact_counts(sorted(list(arm) + list(base)), n1, observed, tol)
    else:
        method, total = "monte_carlo", mc_shuffles
        ge, tie = _mc_counts(arm, base, observed, tol, mc_shuffles, seed)
    return {"method": method, "total": total, "count_ge": ge, "count_tie_at_observed": tie,
            "observed_stat": observed, "p": ge / total, "p_strict": (ge - tie) / total}


def _first_primes(k):
    out, c = [], 2
    while len(out) < k:
        if all(c % q for q in out if q * q <= c):
            out.append(c)
        c += 1
    return out


def separation_floor(n, exact_max=400000, tol=TOL):
    """Permutation result for two generic samples of size n that do not overlap at all (the most extreme
    outcome). Irregular values (logs of distinct primes) avoid accidental ties. None when not exact."""
    if math.comb(2 * n, n) > exact_max:
        return None
    pr = _first_primes(2 * n)
    return permutation_test([math.log(q) for q in pr[:n]], [1000.0 + math.log(q) for q in pr[n:]],
                            exact_max=exact_max, tol=tol)


# --------------------------------------------------------------------------- bootstrap, Holm, binomial

def bootstrap_ratio_ci(arm, base, resamples=10000, seed=12345, level=0.95):
    """Percentile bootstrap CI of median(arm)/median(base); each sample resampled independently."""
    rng = random.Random(seed)
    choices = rng.choices
    na, nb = len(arm), len(base)
    ratios = []
    for _ in range(resamples):
        ma = median_sorted(sorted(choices(arm, k=na)))
        mb = median_sorted(sorted(choices(base, k=nb)))
        if mb != 0.0:
            ratios.append(ma / mb)
    if not ratios:
        return None, None, 0
    ratios.sort()
    alpha = 1.0 - level
    return quantile_linear(ratios, alpha / 2), quantile_linear(ratios, 1 - alpha / 2), len(ratios)


def holm_adjust(pvals):
    """Holm-Bonferroni adjusted p-values, same order as the input (R's p.adjust(method='holm'))."""
    m = len(pvals)
    order = sorted(range(m), key=lambda i: pvals[i])
    adj = [None] * m
    running = 0.0
    for rank, i in enumerate(order):
        running = max(running, (m - rank) * pvals[i])
        adj[i] = min(1.0, running)
    return adj


def binom_pmf(k, n, p):
    return math.comb(n, k) * p ** k * (1.0 - p) ** (n - k)


def binom_sf(k, n, p):
    """P(X >= k), X ~ Binomial(n, p)."""
    return math.fsum(binom_pmf(i, n, p) for i in range(max(k, 0), n + 1))


def binom_cdf(k, n, p):
    """P(X <= k), X ~ Binomial(n, p)."""
    return math.fsum(binom_pmf(i, n, p) for i in range(0, min(k, n) + 1))


def clopper_pearson(k, n, alpha=0.05):
    """Exact (Clopper-Pearson) two-sided CI for a binomial proportion, by bisection."""
    a = alpha / 2.0
    if k <= 0:
        lo = 0.0
    else:
        l, h = 0.0, 1.0
        for _ in range(200):
            mid = (l + h) / 2.0
            if binom_sf(k, n, mid) < a:
                l = mid
            else:
                h = mid
        lo = (l + h) / 2.0
    if k >= n:
        up = 1.0
    else:
        l, h = 0.0, 1.0
        for _ in range(200):
            mid = (l + h) / 2.0
            if binom_cdf(k, n, mid) > a:
                l = mid
            else:
                h = mid
        up = (l + h) / 2.0
    return lo, up


def _compare_worker(task):
    key, arm, base, cfg = task
    perm = permutation_test(arm, base, cfg["exact_max"], cfg["mc"], cfg["seed"])
    lo, hi, n_used = bootstrap_ratio_ci(arm, base, cfg["boot"], cfg["seed"])
    return key, perm, (lo, hi, n_used)


# --------------------------------------------------------------------------- loading

def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def discover(results_dir, baseline, datasets=None, arms=None):
    found = {}
    for name in sorted(os.listdir(results_dir)):
        path = os.path.join(results_dir, name, "results.json")
        if "_" in name and os.path.isfile(path):
            ds, arm = name.split("_", 1)
            found[(ds, arm)] = path
    ds_all = sorted({ds for ds, arm in found if arm == baseline},
                    key=lambda d: (DATASET_ORDER.index(d) if d in DATASET_ORDER else 99, d))
    if datasets:
        ds_all = [d for d in ds_all if d in datasets]
    arms_by_ds = {}
    for ds in ds_all:
        present = sorted({arm for d, arm in found if d == ds and arm != baseline},
                         key=lambda a: (ARM_ORDER.index(a) if a in ARM_ORDER else 99, a))
        arms_by_ds[ds] = [a for a in present if not arms or a in arms]
    return found, ds_all, arms_by_ds


def _dataset_label(doc, arm):
    """Dataset of a flat-layout results.json: meta.dataset (run_interleaved.py), else the first word
    of its config name when that is a known dataset (euroc_stereo -> euroc), else 'all'."""
    meta = doc.get("meta") if isinstance(doc.get("meta"), dict) else {}
    if meta.get("dataset"):
        return str(meta["dataset"])
    head = str(doc.get("config", arm)).split("_", 1)[0]
    return head if head in DATASET_ORDER else "all"


def discover_flat(results_dir, baseline, arms=None, extra=()):
    """Flat layout: results_dir/<arm>/results.json, arm names free, one dataset.

    Returns (found, datasets, arms_by_ds) like discover(); found also holds the arms named only in
    --pair (extra) so they get loaded, arms_by_ds lists just the arms compared with the baseline.
    """
    present = {n: os.path.join(results_dir, n, "results.json") for n in sorted(os.listdir(results_dir))
               if os.path.isfile(os.path.join(results_dir, n, "results.json"))}
    if baseline not in present:
        sys.exit("no %s/results.json under %s (arms found: %s)" % (baseline, results_dir, ", ".join(present) or "none"))
    wanted = [a for a in (arms if arms else [n for n in present if n != baseline]) if a != baseline]
    for a in list(wanted) + list(extra):
        if a not in present:
            sys.exit("no %s/results.json under %s (arms found: %s)" % (a, results_dir, ", ".join(present)))
    labels = {}
    for a in [baseline] + wanted + [x for x in extra if x != baseline and x not in wanted]:
        with open(present[a], encoding="utf-8") as f:
            labels[a] = _dataset_label(json.load(f), a)
    ds = labels[baseline]
    odd = {a: d for a, d in labels.items() if d != ds}
    if odd:
        sys.exit("flat layout compares arms of one dataset; baseline %s is %s but %s" % (
            baseline, ds, ", ".join("%s is %s" % kv for kv in odd.items())))
    found = {(ds, a): present[a] for a in labels}
    return found, [ds], {ds: wanted}


def get_path(obj, path):
    """obj['a']['b'] for 'a.b'; a plain key is looked up first so legacy names keep working. None if absent."""
    if isinstance(obj, dict) and path in obj:
        return obj[path]
    cur = obj
    for part in path.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return None
        cur = cur[part]
    return cur


def _tail_lines(path, n=2, maxbytes=4096, width=240):
    try:
        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - maxbytes))
            data = f.read()
    except OSError:
        return None
    lines = [ln.strip() for ln in data.decode("utf-8", "replace").splitlines() if ln.strip()]
    return [ln[:width] for ln in lines[-n:]]


def _find_log(results_dir, log):
    if not log:
        return None
    for root in (os.path.dirname(os.path.abspath(results_dir)), os.path.abspath(results_dir)):
        p = os.path.join(root, log)
        if os.path.isfile(p):
            return p
    # relative to the repo root of whoever wrote it, which need not be an ancestor of results_dir:
    # the longest tail of the path that exists under results_dir (flat layout: <arm>/<seq>/runNN/slam.log)
    parts = [x for x in log.replace("\\", "/").split("/") if x not in ("", ".", "..")]
    for i in range(len(parts)):
        p = os.path.join(os.path.abspath(results_dir), *parts[i:])
        if os.path.isfile(p):
            return p
    return None


def _finite(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def make_group(ds, arm, seq, metric, doc, results_dir):
    sd = doc["sequences"][seq]
    runs = sd.get("runs", [])
    vals, dropped = [], []
    for r in runs:
        v = get_path(r, metric)
        if r.get("status") == "ok" and _finite(v):
            vals.append({"run": r.get("run"), "value": float(v), "coverage": r.get("coverage"),
                         "seconds": r.get("seconds")})
        else:
            lp = _find_log(results_dir, r.get("log"))
            dropped.append({"run": r.get("run"), "status": r.get("status"),
                            "returncode": r.get("returncode"), "seconds": r.get("seconds"),
                            "log": r.get("log"), "value_in_json": get_path(r, metric),
                            "log_tail": _tail_lines(lp) if lp else None})
    g = {"dataset": ds, "arm": arm, "sequence": seq, "metric": metric,
         "runs_declared": doc.get("runs"), "n_listed": len(runs), "n_ok": len(vals),
         "n_dropped": len(dropped), "dropped": dropped, "values": vals}
    xs = [e["value"] for e in vals]
    if xs:
        s = sorted(xs)
        g.update(median=median_sorted(s), mean=mean(xs), std=stdev(xs), min=s[0], max=s[-1],
                 q1=quantile_linear(s, 0.25), q3=quantile_linear(s, 0.75),
                 max_over_min=(s[-1] / s[0]) if s[0] > 0 else None)
        g["duplicate_values"] = len(xs) - len(set(xs))
        iqr = g["q3"] - g["q1"]
        outs = []
        if iqr > 0:
            for e in vals:
                v = e["value"]
                for k, extreme_k in ((1.5, 3.0),):
                    lo_f, hi_f = g["q1"] - k * iqr, g["q3"] + k * iqr
                    if v < lo_f or v > hi_f:
                        outs.append({"run": e["run"], "value": v, "side": "high" if v > hi_f else "low",
                                     "extreme": bool(v < g["q1"] - extreme_k * iqr
                                                     or v > g["q3"] + extreme_k * iqr)})
        g["outliers"] = outs
        cov = [e["coverage"] for e in vals if _finite(e["coverage"])]
        g["coverage"] = ({"min": min(cov), "median": median(cov), "max": max(cov)} if cov else None)
        sec = [e["seconds"] for e in vals if _finite(e["seconds"])]
        g["seconds"] = ({"min": min(sec), "median": median(sec), "max": max(sec)} if sec else None)
    agg = sd.get(metric) if isinstance(sd.get(metric), dict) else None
    if agg and xs:
        g["aggregate_check"] = {
            "json": {k: agg.get(k) for k in ("n", "median", "mean", "std")},
            "ok": bool(agg.get("n") == len(xs)
                       and all((agg.get(k) is None and g.get(k) is None)  # std of ONE run is None on both sides
                               or (_finite(agg.get(k)) and _finite(g.get(k))
                                   and math.isclose(agg[k], g[k], rel_tol=1e-9, abs_tol=1e-12))
                               for k in ("median", "mean", "std")))}
    return g


# --------------------------------------------------------------------------- comparisons

def _fill(recs, tasks, jobs):
    """Run the permutation test and the bootstrap of every task and write the numbers into recs."""
    if jobs > 1:
        from concurrent.futures import ProcessPoolExecutor
        with ProcessPoolExecutor(max_workers=jobs) as ex:
            results = list(ex.map(_compare_worker, tasks, chunksize=1))
    else:
        results = [_compare_worker(t) for t in tasks]
    for key, perm, (lo, hi, n_used) in results:
        r = recs[key]
        g, b = r.pop("_g"), r.pop("_b")
        r.update(n_arm_ok=g["n_ok"], n_arm_failed=g["n_dropped"], n_base_ok=b["n_ok"],
                 n_base_failed=b["n_dropped"],
                 arm_median=g["median"], arm_mean=g["mean"], arm_std=g["std"], arm_min=g["min"], arm_max=g["max"],
                 base_median=b["median"], base_mean=b["mean"], base_std=b["std"], base_min=b["min"], base_max=b["max"])
        r["diff_median"] = g["median"] - b["median"]
        r["ratio_median"] = g["median"] / b["median"] if b["median"] != 0 else None
        r["rel_change_median_pct"] = (r["ratio_median"] - 1.0) * 100.0 if r["ratio_median"] is not None else None
        r["ratio_ci95_lo"], r["ratio_ci95_hi"] = lo, hi
        r["bootstrap_resamples_used"] = n_used
        r["direction"] = ("tie" if abs(r["diff_median"]) <= TOL
                          else "better" if r["diff_median"] < 0 else "worse")
        r["perm"] = {k: perm[k] for k in ("method", "total", "count_ge", "count_tie_at_observed", "observed_stat")}
        r["p_raw"] = perm["p"]
        r["p_strict_sensitivity"] = perm["p_strict"]      # ties at the observed value counted as NOT extreme
        r["tie_fraction_at_observed"] = perm["count_tie_at_observed"] / perm["total"]


def run_comparisons(groups, datasets, arms_by_ds, seq_by_ds, baseline, cfg, jobs):
    recs, tasks = [], []
    for ds in datasets:
        for metric in cfg["metrics"]:
            for seq in seq_by_ds[ds]:
                base = groups.get((ds, baseline, seq, metric))
                if base is None or base["n_ok"] == 0:
                    continue
                for arm in arms_by_ds[ds]:
                    g = groups.get((ds, arm, seq, metric))
                    if g is None or g["n_ok"] == 0:
                        continue
                    key = len(recs)
                    xa = [e["value"] for e in g["values"]]
                    xb = [e["value"] for e in base["values"]]
                    recs.append({"dataset": ds, "sequence": seq, "arm": arm, "baseline_arm": baseline,
                                 "metric": metric, "_g": g, "_b": base})
                    tasks.append((key, xa, xb, cfg))
    _fill(recs, tasks, jobs)
    # Holm: whole family (all comparisons of a metric) and within each dataset
    for metric in cfg["metrics"]:
        fam = [r for r in recs if r["metric"] == metric]
        if not fam:
            continue
        for r, a in zip(fam, holm_adjust([r["p_raw"] for r in fam])):
            r["p_holm_family"] = a
            r["family_size"] = len(fam)
        for ds in datasets:
            sub = [r for r in fam if r["dataset"] == ds]
            for r, a in zip(sub, holm_adjust([r["p_raw"] for r in sub])):
                r["p_holm_dataset"] = a
                r["dataset_family_size"] = len(sub)
    # sensitivity: the same Holm family without the null-control arms (only the real contrasts)
    for metric in cfg["metrics"]:
        fam = [r for r in recs if r["metric"] == metric]
        sub = [r for r in fam if r["arm"] not in cfg["null_arms"]]
        for r in fam:
            r["p_holm_family_excl_null"], r["family_size_excl_null"] = None, None
        for r, a in zip(sub, holm_adjust([r["p_raw"] for r in sub])):
            r["p_holm_family_excl_null"] = a
            r["family_size_excl_null"] = len(sub)
    thr = cfg["regression_ratio"]
    for r in recs:
        a = cfg["alpha"]
        r["sig_raw"] = r["p_raw"] < a
        r["sig_holm_dataset"] = r["p_holm_dataset"] < a
        r["sig_holm_family"] = r["p_holm_family"] < a
        r["ci_hi_gt_regression_ratio"] = bool(r["ratio_ci95_hi"] is not None and r["ratio_ci95_hi"] > thr)
    return recs


def run_pair_comparisons(groups, datasets, seq_by_ds, pairs, cfg, jobs):
    """ARM against REF for every (REF, ARM) pair, dataset, metric and sequence. Own Holm family per metric."""
    recs, tasks = [], []
    for ref, arm in pairs:
        for ds in datasets:
            for metric in cfg["metrics"]:
                for seq in seq_by_ds[ds]:
                    base = groups.get((ds, ref, seq, metric))
                    g = groups.get((ds, arm, seq, metric))
                    if base is None or g is None or base["n_ok"] == 0 or g["n_ok"] == 0:
                        continue
                    key = len(recs)
                    recs.append({"dataset": ds, "sequence": seq, "arm": arm, "baseline_arm": ref, "pair": "%s:%s" % (ref, arm),
                                 "metric": metric, "_g": g, "_b": base})
                    tasks.append((key, [e["value"] for e in g["values"]], [e["value"] for e in base["values"]], cfg))
    _fill(recs, tasks, jobs)
    for metric in cfg["metrics"]:
        fam = [r for r in recs if r["metric"] == metric]
        for r, a in zip(fam, holm_adjust([r["p_raw"] for r in fam])):
            r["p_holm_pairs"] = a
            r["pair_family_size"] = len(fam)
    for r in recs:
        r["sig_raw"] = r["p_raw"] < cfg["alpha"]
        r["sig_holm_pairs"] = r["p_holm_pairs"] < cfg["alpha"]
        r["ci_hi_gt_regression_ratio"] = bool(r["ratio_ci95_hi"] is not None and r["ratio_ci95_hi"] > cfg["regression_ratio"])
    return recs


# --------------------------------------------------------------------------- summaries (a)-(d)

def _brief(r):
    return {"dataset": r["dataset"], "sequence": r["sequence"], "arm": r["arm"], "metric": r["metric"],
            "rel_change_median_pct": r["rel_change_median_pct"], "ratio_median": r["ratio_median"],
            "ratio_ci95": [r["ratio_ci95_lo"], r["ratio_ci95_hi"]], "p_raw": r["p_raw"],
            "p_holm_dataset": r["p_holm_dataset"], "p_holm_family": r["p_holm_family"]}


def count_block(rows, alpha, null_rate=None):
    m = len(rows)
    k = sum(r["sig_raw"] for r in rows)
    return {"n_comparisons": m,
            "raw_lt_alpha": k,
            "raw_lt_alpha_better": sum(r["sig_raw"] and r["direction"] == "better" for r in rows),
            "raw_lt_alpha_worse": sum(r["sig_raw"] and r["direction"] == "worse" for r in rows),
            "holm_dataset_lt_alpha": sum(r["sig_holm_dataset"] for r in rows),
            "holm_family_lt_alpha": sum(r["sig_holm_family"] for r in rows),
            "holm_family_excl_null_lt_alpha": (
                sum(1 for r in rows if r.get("p_holm_family_excl_null") is not None and r["p_holm_family_excl_null"] < alpha)
                if any(r.get("p_holm_family_excl_null") is not None for r in rows) else None),
            "expected_raw_by_chance": alpha * m,
            "expected_raw_at_null_control_rate": (null_rate * m) if null_rate is not None else None,
            "binomial_P_X_ge_k_under_null": binom_sf(k, m, alpha) if m else None,
            "P_at_least_one_raw_under_global_null": (1 - (1 - alpha) ** m) if m else None}


def summaries(recs, datasets, arms, null_arms, cfg):
    alpha, reg = cfg["alpha"], cfg["regression_ratio"]
    out = {"alpha": alpha, "regression_ratio": reg, "notes": [
        "expected_raw_by_chance = alpha * m (the exact permutation test is conservative, so the true "
        "expectation under the null is <= this).",
        "binomial_P_X_ge_k_under_null assumes independent comparisons; the three arms of a sequence "
        "share one baseline sample, so they are positively correlated and the tail is only indicative.",
        "Under the global null Holm controls the family-wise error: P(at least one Holm p < alpha) <= alpha."]}
    for metric in cfg["metrics"]:
        rows = [r for r in recs if r["metric"] == metric]
        if not rows:
            continue
        nrows = [r for r in rows if r["arm"] in null_arms]
        nrate = (sum(r["sig_raw"] for r in nrows) / len(nrows)) if nrows else None
        m = {"overall": count_block(rows, alpha, nrate), "by_arm": {}, "by_dataset_arm": {},
             "null_control_rate_used_for_expected": nrate}
        for arm in arms:
            m["by_arm"][arm] = count_block([r for r in rows if r["arm"] == arm], alpha, nrate)
        # the two real contrasts together (null-control arm excluded)
        real = [r for r in rows if r["arm"] not in null_arms]
        m["non_null_arms_combined"] = count_block(real, alpha, nrate)
        for ds in datasets:
            for arm in arms:
                sub = [r for r in rows if r["dataset"] == ds and r["arm"] == arm]
                if sub:
                    m["by_dataset_arm"][f"{ds}/{arm}"] = count_block(sub, alpha, nrate)
        # (c) lists
        lists = {}
        for arm in arms:
            ar = [r for r in rows if r["arm"] == arm]
            lists[arm] = {
                "better_raw": [_brief(r) for r in ar if r["sig_raw"] and r["direction"] == "better"],
                "worse_raw": [_brief(r) for r in ar if r["sig_raw"] and r["direction"] == "worse"],
                "better_holm_family": [_brief(r) for r in ar if r["sig_holm_family"] and r["direction"] == "better"],
                "worse_holm_family": [_brief(r) for r in ar if r["sig_holm_family"] and r["direction"] == "worse"],
                "better_holm_dataset": [_brief(r) for r in ar if r["sig_holm_dataset"] and r["direction"] == "better"],
                "worse_holm_dataset": [_brief(r) for r in ar if r["sig_holm_dataset"] and r["direction"] == "worse"],
                "ratio_ci_upper_gt_%.2f" % reg: [_brief(r) for r in ar if r["ci_hi_gt_regression_ratio"]]}
        m["lists_c"] = lists
        # (d) wins / losses / ties
        wl = {}
        for ds in datasets + ["ALL"]:
            for arm in arms:
                sub = [r for r in rows if r["arm"] == arm and (ds == "ALL" or r["dataset"] == ds)]
                if not sub:
                    continue
                def wlt(pred_w, pred_l):
                    w = sum(1 for r in sub if pred_w(r))
                    l = sum(1 for r in sub if pred_l(r))
                    return {"wins": w, "losses": l, "ties": len(sub) - w - l}
                wl[f"{ds}/{arm}"] = {
                    "n": len(sub),
                    "by_median_sign": wlt(lambda r: r["direction"] == "better", lambda r: r["direction"] == "worse"),
                    "by_raw_p": wlt(lambda r: r["sig_raw"] and r["direction"] == "better",
                                    lambda r: r["sig_raw"] and r["direction"] == "worse"),
                    "by_holm_dataset": wlt(lambda r: r["sig_holm_dataset"] and r["direction"] == "better",
                                           lambda r: r["sig_holm_dataset"] and r["direction"] == "worse"),
                    "by_holm_family": wlt(lambda r: r["sig_holm_family"] and r["direction"] == "better",
                                          lambda r: r["sig_holm_family"] and r["direction"] == "worse")}
        m["wins_losses_ties_d"] = wl
        out[metric] = m
    # resolution of the exact test and Holm's first-rank threshold
    floor = []
    for (n1, n2) in sorted({(r["n_arm_ok"], r["n_base_ok"]) for r in recs}):
        cnt = sum(1 for r in recs if (r["n_arm_ok"], r["n_base_ok"]) == (n1, n2))
        entry = {"n_arm": n1, "n_base": n2, "n_comparisons": cnt}
        if n1 == n2:
            sf = separation_floor(n1, cfg["exact_max"])
            if sf:
                entry.update(total_relabelings=sf["total"], count_at_complete_separation=sf["count_ge"],
                             p_complete_separation=sf["p"])
        else:
            entry["note"] = "unequal sizes: the smallest p depends on the spacing of the data, not computed"
        floor.append(entry)
    fam = {}
    for metric in cfg["metrics"]:
        rows = [r for r in recs if r["metric"] == metric]
        if rows:
            fam[metric] = {"family": alpha / len(rows),
                           "by_dataset": {ds: alpha / sum(1 for r in rows if r["dataset"] == ds)
                                          for ds in datasets if any(r["dataset"] == ds for r in rows)}}
    out["p_resolution"] = {
        "equal_size_complete_separation": floor,
        "holm_first_rank_raw_p_threshold": fam,
        "min_p_raw_observed": {m: min(r["p_raw"] for r in recs if r["metric"] == m) for m in cfg["metrics"]
                               if any(r["metric"] == m for r in recs)},
        "n_comparisons_at_min_p_raw": {m: sum(1 for r in recs if r["metric"] == m and r["p_raw"] == min(
            x["p_raw"] for x in recs if x["metric"] == m)) for m in cfg["metrics"] if any(r["metric"] == m for r in recs)},
        "note": "Exact-test p-values are multiples of 1/total_relabelings. Holm-adjusted p < alpha needs raw p <= "
                "alpha/(m - rank + 1); holm_first_rank_raw_p_threshold is that bound for the top-ranked comparison."}
    # (b) null control
    if any(a in arms for a in null_arms):
        nb = {}
        for metric in cfg["metrics"]:
            rows = [r for r in recs if r["metric"] == metric and r["arm"] in null_arms]
            if not rows:
                continue
            k = sum(r["sig_raw"] for r in rows)
            lo, hi = clopper_pearson(k, len(rows), alpha)
            ab = sorted(abs(r["rel_change_median_pct"]) for r in rows)
            signed = sorted(r["rel_change_median_pct"] for r in rows)
            nb[metric] = {
                "arm": ",".join(null_arms), "arms": list(null_arms), "n_comparisons": len(rows), "raw_lt_alpha": k,
                "false_positive_rate": k / len(rows), "clopper_pearson_95": [lo, hi],
                "expected_by_chance": alpha * len(rows),
                "binomial_P_X_ge_k_under_null": binom_sf(k, len(rows), alpha),
                "by_dataset": {ds: {"n": sum(1 for r in rows if r["dataset"] == ds),
                                    "raw_lt_alpha": sum(1 for r in rows if r["dataset"] == ds and r["sig_raw"])}
                               for ds in datasets if any(r["dataset"] == ds for r in rows)},
                "sequences_raw_lt_alpha": [_brief(r) for r in rows if r["sig_raw"]],
                "holm_family_lt_alpha": sum(r["sig_holm_family"] for r in rows),
                "holm_dataset_lt_alpha": sum(r["sig_holm_dataset"] for r in rows),
                "p_raw_sorted": sorted(r["p_raw"] for r in rows),
                "ratio_ci_excludes_1": sum(1 for r in rows if not (r["ratio_ci95_lo"] <= 1.0 <= r["ratio_ci95_hi"])),
                "ratio_ci_upper_gt_regression": sum(r["ci_hi_gt_regression_ratio"] for r in rows),
                "abs_rel_change_pct": {"min": ab[0], "median": median_sorted(ab), "max": ab[-1]},
                "signed_rel_change_pct": {"min": signed[0], "median": median_sorted(signed), "max": signed[-1]},
                "direction_counts": {"better": sum(r["direction"] == "better" for r in rows),
                                     "worse": sum(r["direction"] == "worse" for r in rows),
                                     "tie": sum(r["direction"] == "tie" for r in rows)},
                "per_comparison": [_brief(r) for r in rows]}
        out["b_null_control"] = nb
    return out


def pair_summary(pair_recs, pairs, cfg):
    alpha = cfg["alpha"]
    out = {}
    for metric in cfg["metrics"]:
        rows = [r for r in pair_recs if r["metric"] == metric]
        if not rows:
            continue
        out[metric] = {
            "n_comparisons": len(rows), "raw_lt_alpha": sum(r["sig_raw"] for r in rows),
            "raw_lt_alpha_better": sum(r["sig_raw"] and r["direction"] == "better" for r in rows),
            "raw_lt_alpha_worse": sum(r["sig_raw"] and r["direction"] == "worse" for r in rows),
            "holm_pairs_lt_alpha": sum(r["sig_holm_pairs"] for r in rows),
            "expected_raw_by_chance": alpha * len(rows),
            "by_pair": {"%s:%s" % pr: {"n_comparisons": sum(1 for r in rows if r["pair"] == "%s:%s" % pr),
                                       "raw_lt_alpha": sum(r["sig_raw"] for r in rows if r["pair"] == "%s:%s" % pr)}
                        for pr in pairs}}
    return out


# --------------------------------------------------------------------------- run-order diagnostics

def _ranks(v):
    order = sorted(range(len(v)), key=v.__getitem__)
    r = [0.0] * len(v)
    i = 0
    while i < len(v):
        j = i
        while j + 1 < len(v) and v[order[j + 1]] == v[order[i]]:
            j += 1
        for k in range(i, j + 1):
            r[order[k]] = (i + j) / 2.0 + 1.0
        i = j + 1
    return r


def _pearson(x, y):
    mx, my = mean(x), mean(y)
    sx = math.sqrt(math.fsum((a - mx) ** 2 for a in x))
    sy = math.sqrt(math.fsum((b - my) ** 2 for b in y))
    if sx == 0 or sy == 0:
        return 0.0
    return math.fsum((a - mx) * (b - my) for a, b in zip(x, y)) / (sx * sy)


def run_order_diagnostics(groups, seed, resamples):
    """Do runs inside one block look exchangeable? Pooled over all ATE groups with n_ok >= 4.

    trend = Spearman(run index, value); lag1 = lag-1 autocorrelation of the ranks of consecutive ok runs.
    Each is averaged over groups; the null distribution comes from shuffling the values inside every group
    (random.Random(seed)). Two-sided MC p-values are (#|stat - null mean| >= |obs - null mean|) / resamples.
    """
    items = []
    for g in groups.values():
        if g["metric"] != "ate_rmse" or g["n_ok"] < 4:
            continue
        idx = []
        for pos, e in enumerate(g["values"]):
            digits = "".join(ch for ch in str(e["run"]) if ch.isdigit())
            idx.append(int(digits) if digits else pos)
        items.append((g["arm"], idx, [e["value"] for e in g["values"]]))
    if not items or resamples <= 0:
        return None

    def stat(pairs):
        tr, lg = [], []
        for idx, v in pairs:
            rk = _ranks(v)
            tr.append(_pearson(idx, rk))
            lg.append(_pearson(rk[:-1], rk[1:]))
        return tr, lg

    tr, lg = stat([(i, v) for _, i, v in items])
    obs_t, obs_a = mean(tr), mean(lg)
    rng = random.Random(seed)
    nt, na = [], []
    for _ in range(resamples):
        sh = []
        for _, i, v in items:
            w = list(v)
            rng.shuffle(w)
            sh.append((i, w))
        a, b = stat(sh)
        nt.append(mean(a))
        na.append(mean(b))
    mt, ma = mean(nt), mean(na)
    first = []
    for _, i, v in items:
        if 0 in i and len(v) > 1:
            first.append((_ranks(v)[i.index(0)] - 1.0) / (len(v) - 1.0))
    arms = sorted({a for a, _, _ in items})
    return {"n_groups": len(items), "resamples": resamples,
            "mean_spearman_run_index_vs_value": obs_t, "null_mean_spearman": mt,
            "p_trend_mc": sum(abs(x - mt) >= abs(obs_t - mt) - 1e-12 for x in nt) / resamples,
            "mean_lag1_rank_autocorrelation": obs_a, "null_mean_lag1": ma,
            "p_lag1_mc": sum(abs(x - ma) >= abs(obs_a - ma) - 1e-12 for x in na) / resamples,
            "mean_normalised_rank_of_run00": mean(first) if first else None, "n_groups_with_run00": len(first),
            "by_arm": {a: {"mean_spearman": mean([t for (ar, _, _), t in zip(items, tr) if ar == a]),
                           "mean_lag1": mean([t for (ar, _, _), t in zip(items, lg) if ar == a]),
                           "n_groups": sum(1 for ar, _, _ in items if ar == a)} for a in arms}}


# --------------------------------------------------------------------------- flags

def build_flags(groups, recs, datasets, arms_by_ds, baseline, cfg):
    flags = []

    def add(kind, severity, where, detail, **data):
        flags.append({"type": kind, "severity": severity, "where": where, "detail": detail, "data": data})

    gl = list(groups.values())
    for g in gl:
        tag = f"{arm_dir(cfg, g['dataset'], g['arm'])}/{g['sequence']}"
        for d in g["dropped"]:
            if g["metric"] != "ate_rmse" and d["status"] != "ok":
                continue                       # already reported through the ATE group
            if d["status"] == "ok":
                add("ok_run_invalid_value", "warn", f"{tag}/{d['run']}",
                    f"status ok but {g['metric']} = {d['value_in_json']!r}: dropped from {g['metric']}")
                continue
            add("failed_run", "warn", f"{tag}/{d['run']}",
                f"dropped: status={d['status']} returncode={d['returncode']} seconds={d['seconds']}"
                + (" (also dropped from t_rel_pct)" if g["dataset"] == "kitti" else ""),
                log_tail=d.get("log_tail"), metric=g["metric"])
        if g["metric"] == "ate_rmse":
            if g["runs_declared"] and g["n_ok"] < g["runs_declared"]:
                add("n_ok_below_declared", "warn", tag, f"n_ok={g['n_ok']} of {g['runs_declared']} declared runs")
            if g["n_ok"] == 0:
                continue
            if g["duplicate_values"]:
                add("exact_ties", "warn", tag, f"{g['duplicate_values']} duplicate value(s) within the group")
            if g["outliers"]:
                add("outliers", "info", tag, "; ".join(
                    "%s=%.5g (%s%s)" % (o["run"], o["value"], o["side"], ", >3xIQR" if o["extreme"] else "")
                    for o in g["outliers"]), n=len(g["outliers"]),
                    extreme=sum(o["extreme"] for o in g["outliers"]))
            if g["max_over_min"] and g["max_over_min"] >= cfg["heavy_spread"]:
                add("heavy_spread", "info", tag, "max/min = %.2f (min %.5g, median %.5g, max %.5g)" % (
                    g["max_over_min"], g["min"], g["median"], g["max"]))
            cov = g.get("coverage")
            if cov:
                if cov["median"] < 0.5:
                    add("coverage_low", "warn", tag, "median coverage %.3f: ATE is computed on only %.1f%% of the "
                        "estimated poses" % (cov["median"], 100 * cov["median"]))
                if cov["max"] - cov["min"] > cfg["coverage_tol"]:
                    odd = [(e["run"], e["coverage"], e["value"]) for e in g["values"]
                           if _finite(e["coverage"]) and abs(e["coverage"] - cov["median"]) > cfg["coverage_tol"]]
                    add("coverage_varies_within_group", "warn", tag,
                        "coverage %.3f-%.3f; runs off the median: %s" % (cov["min"], cov["max"], ", ".join(
                            "%s cov=%.3f ate=%.5g" % o for o in odd)), runs=[o[0] for o in odd])
            sec = g.get("seconds")
            if sec and sec["median"] > 0:
                slow = [(e["run"], e["seconds"], e["value"]) for e in g["values"]
                        if _finite(e["seconds"]) and e["seconds"] > cfg["walltime_ratio"] * sec["median"]]
                if slow:
                    add("walltime_outlier", "info", tag, "wall time > %.1fx the group median (%.1f s): %s" % (
                        cfg["walltime_ratio"], sec["median"], ", ".join("%s %.1fs ate=%.5g" % s for s in slow)),
                        runs=[s[0] for s in slow])
        ac = g.get("aggregate_check")
        if ac and not ac["ok"]:
            add("aggregate_mismatch", "warn", f"{tag}/{g['metric']}",
                "recomputed n/median/mean/std differ from the aggregate stored in results.json", json=ac["json"])
    for r in recs:
        tag = f"{r['dataset']}/{r['sequence']}/{r['arm']}"
        if r["metric"] != "ate_rmse":
            continue
        gb = groups[(r["dataset"], baseline, r["sequence"], "ate_rmse")]
        ga = groups[(r["dataset"], r["arm"], r["sequence"], "ate_rmse")]
        if gb.get("coverage") and ga.get("coverage") and \
                abs(gb["coverage"]["median"] - ga["coverage"]["median"]) > cfg["coverage_tol"]:
            add("coverage_differs_arm_vs_baseline", "warn", tag,
                "median coverage arm %.3f vs baseline %.3f (ATE computed on different pose sets)" % (
                    ga["coverage"]["median"], gb["coverage"]["median"]))
        pooled = [e["value"] for e in ga["values"]] + [e["value"] for e in gb["values"]]
        if len(set(pooled)) < len(pooled):
            add("pooled_exact_ties", "warn", tag, "%d tied values in the pooled sample" % (len(pooled) - len(set(pooled))))
        if r["perm"]["method"] != "exact":
            add("monte_carlo_p", "info", tag, "p-value by Monte Carlo (%d shuffles): resolution %.1e" % (
                r["perm"]["total"], 1.0 / r["perm"]["total"]))
    tf = sorted(r["tie_fraction_at_observed"] for r in recs)
    if tf:
        alpha = cfg["alpha"]
        flip = [r for r in recs if (r["p_raw"] < alpha) != (r["p_strict_sensitivity"] < alpha)]
        add("permutation_tie_mass", "info" if not flip else "warn", "all comparisons",
            "share of relabelings exactly tied with the observed statistic (inherent to a median-difference "
            "statistic, not to data ties): min %.2f%%, median %.2f%%, max %.2f%%; counting those ties as not extreme "
            "(strict >) would change the p<%.2f classification of %d of %d comparisons%s" % (
                100 * tf[0], 100 * median_sorted(tf), 100 * tf[-1], alpha, len(flip), len(recs),
                "" if not flip else ": " + ", ".join("%s/%s/%s %s (p %.4f -> %.4f)" % (
                    r["dataset"], r["sequence"], r["arm"], r["metric"], r["p_raw"], r["p_strict_sensitivity"])
                    for r in flip)))
    for r in recs:
        if r["arm"] not in cfg["null_arms"] or not r["sig_raw"]:
            continue
        others = [o for o in recs if o["dataset"] == r["dataset"] and o["sequence"] == r["sequence"]
                  and o["metric"] == r["metric"] and o["arm"] != r["arm"]]
        add("null_control_hit", "warn", "%s/%s/%s" % (r["dataset"], r["sequence"], r["arm"]),
            "%s: null-control arm differs from baseline by %s%% (p=%.4f); other arms vs baseline at the same "
            "sequence: %s" % (r["metric"], fpct(r["rel_change_median_pct"]), r["p_raw"],
                               ", ".join("%s %s%% (p=%.4f)" % (o["arm"], fpct(o["rel_change_median_pct"]), o["p_raw"])
                                         for o in others)))
    small = [g for g in gl if g["metric"] == "ate_rmse" and g["n_ok"]]
    if small:
        add("small_n", "info", "all groups", "n_ok per group ranges %d-%d; the percentile bootstrap CI of a "
            "median ratio is approximate at this n and the tests have low power" % (
                min(g["n_ok"] for g in small), max(g["n_ok"] for g in small)))
    return flags


# --------------------------------------------------------------------------- markdown

def fnum(x, d=5):
    return "-" if x is None else f"{x:.{d}g}"


def fp(p):
    if p is None:
        return "-"
    if p == 0:
        return "0"
    return f"{p:.1e}" if p < 1e-4 else f"{p:.4f}"


def fpct(x):
    return "-" if x is None else f"{x:+.1f}"


def md_table(headers, rows, aligns=None):
    al = aligns or ["l"] * len(headers)
    sep = {"l": ":---", "r": "---:", "c": ":---:"}
    esc = lambda c: str(c).replace("|", "\\|")
    lines = ["| " + " | ".join(esc(h) for h in headers) + " |", "|" + "|".join(sep[a] for a in al) + "|"]
    lines += ["| " + " | ".join(esc(c) for c in row) + " |" for row in rows]
    return "\n".join(lines)


def mark(r):
    if not r["sig_raw"]:
        return ""
    arrow = {"better": "↓", "worse": "↑"}.get(r["direction"], "=")
    lv = "r" + (",D" if r["sig_holm_dataset"] else "") + (",F" if r["sig_holm_family"] else "")
    return f"{arrow} {lv}"


def lst(items, key_extra=None):
    if not items:
        return "none"
    return "; ".join("%s/%s (%s%%, p=%s, Holm-D=%s, Holm-F=%s)" % (
        i["dataset"], i["sequence"], fpct(i["rel_change_median_pct"]), fp(i["p_raw"]),
        fp(i["p_holm_dataset"]), fp(i["p_holm_family"])) for i in items)


def render_markdown(doc):
    P = doc["meta"]["parameters"]
    recs = doc["comparisons"]
    groups = doc["groups"]
    S = doc["summary"]
    datasets = doc["meta"]["datasets"]
    arms = doc["meta"]["arms"]
    base = doc["meta"]["baseline_arm"]
    metrics = doc["meta"]["metrics"]
    null_label = ", ".join(doc["meta"]["null_arms"])
    flat = doc["meta"]["layout"] == "flat"
    pair_recs = doc["pair_comparisons"]
    gk = {(g["dataset"], g["arm"], g["sequence"], g["metric"]): g for g in groups}
    ck = {(r["dataset"], r["sequence"], r["arm"], r["metric"]): r for r in recs}
    n_ate = sum(1 for r in recs if r["metric"] == "ate_rmse")
    methods = {}
    for r in recs:
        methods[r["perm"]["method"]] = methods.get(r["perm"]["method"], 0) + 1
    L = []
    L.append("# Arm vs baseline statistics")
    L.append("")
    L.append("Generated by `evaluation/harness/stats_compare.py` (schema %d) on %s UTC from `%s`. All numbers in this "
             "file are copied from `stats_all.json`." % (SCHEMA_VERSION, doc["meta"]["created_utc"],
                                                          doc["meta"]["results_dir"]))
    L.append("")
    L.append("## Definitions")
    L.append("")
    L.append("- **Sample**: per dataset/sequence/arm, the metric of every run with `status == ok`; failed runs are dropped "
             "(`n_ok` is reported), never imputed.")
    L.append("- **Test**: arm vs the same dataset's baseline. Statistic = |median(arm) - median(baseline)|. Two-sided "
             "permutation p over relabelings of the pooled values: exact enumeration of all C(n1+n2, n1) relabelings when "
             "<= %d, otherwise Monte Carlo with %d shuffles (seed %d). p = #(statistic >= observed - 1e-12) / total. "
             "Methods used in this run: %s." % (P["perm_exact_max"], P["perm_mc_shuffles"], P["seed"],
                                               ", ".join("%s x %d" % kv for kv in sorted(methods.items()))))
    L.append("- **Holm**: Holm-Bonferroni (step-down, cumulative max). `Holm-F` = over the whole family of %d ATE "
             "comparisons; `Holm-D` = within the dataset (family sizes %s). The KITTI `t_rel_pct` comparisons are a "
             "separate family (their own Holm over %d comparisons)." % (
                 n_ate, ", ".join("%s %d" % (ds, sum(1 for r in recs if r["metric"] == "ate_rmse" and r["dataset"] == ds))
                                  for ds in datasets),
                 sum(1 for r in recs if r["metric"] == "t_rel_pct")))
    extra_metrics = [m for m in metrics if m not in METRICS]
    if extra_metrics:
        L.append("- **Extra metrics** (`--metrics`): %s. Each is its own Holm family (comparisons: %s) and is treated as a "
                 "cost, lower = better." % (", ".join("`%s`" % m for m in extra_metrics),
                                            ", ".join("%d" % sum(1 for r in recs if r["metric"] == m) for m in extra_metrics)))
    if flat:
        L.append("- **Layout**: flat (`<arm>/results.json`, run_interleaved.py). Baseline `%s`; arms compared with it: %s. "
                 "The arms were run interleaved, so a drift over time hits all of them alike." % (
                     base, ", ".join("`%s`" % a for a in arms)))
    L.append("- **Ratio** = median(arm)/median(baseline); 95%% percentile bootstrap CI, %d resamples (each sample "
             "resampled independently with replacement, `random.Random(%d)` re-seeded per comparison, linear-interpolated "
             "percentiles). `R` flags a CI upper bound > %.2f." % (P["bootstrap_resamples"], P["seed"], P["regression_ratio"]))
    L.append("- **Direction**: lower median than baseline = better (every metric is an error). Mark column: ↓ better / ↑ "
             "worse, then the strictest level reached: `r` raw p<%.2f, `D` Holm-D p<%.2f, `F` Holm-F p<%.2f." % (
                 P["alpha"], P["alpha"], P["alpha"]))
    L.append("- `std` is the sample standard deviation (ddof=1). Values use 5 significant digits.")
    L.append("")

    # overview
    ov = "ate_rmse" if "ate_rmse" in metrics else metrics[0]
    ov_name = "ATE" if ov == "ate_rmse" else mlabel(ov)
    L.append("## Overview: change of the median %s vs baseline, %% (raw p)" % ov_name)
    L.append("")
    L.append("Negative = lower %s than baseline. Marks as in the detailed tables; `R` = the 95%% bootstrap CI of the "
             "ratio has an upper bound > %.2f." % (ov_name, P["regression_ratio"]))
    L.append("")
    rows = []
    for ds in datasets:
        for seq in doc["meta"]["sequences"][ds]:
            row = [f"{ds}/{seq}"]
            for arm in arms:
                r = ck.get((ds, seq, arm, ov))
                if r is None:
                    row.append("-")
                else:
                    m = mark(r)
                    row.append("%s (%s)%s%s" % (fpct(r["rel_change_median_pct"]), fp(r["p_raw"]),
                                                (" " + m) if m else "", " R" if r["ci_hi_gt_regression_ratio"] else ""))
            rows.append(row)
    L.append(md_table(["dataset/sequence"] + arms, rows, ["l"] + ["r"] * len(arms)))
    L.append("")

    # detailed tables
    for metric in metrics:
        if not any(r["metric"] == metric for r in recs):
            continue
        for ds in datasets:
            if not any(r["metric"] == metric and r["dataset"] == ds for r in recs):
                continue
            L.append("## %s, %s" % (mlabel(metric), ds))
            L.append("")
            rows = []
            for seq in doc["meta"]["sequences"][ds]:
                for arm in [base] + arms:
                    g = gk.get((ds, arm, seq, metric))
                    if g is None or g["n_ok"] == 0:
                        continue
                    n_txt = str(g["n_ok"]) + (" (%d failed)" % g["n_dropped"] if g["n_dropped"] else "")
                    row = [seq, arm, n_txt, fnum(g["median"]), fnum(g["mean"]), fnum(g["std"])]
                    r = ck.get((ds, seq, arm, metric))
                    if r is None:
                        row += ["", "", "", "", "", "", ""]
                    else:
                        row += [fpct(r["rel_change_median_pct"]),
                                "%.3f [%.3f, %.3f]" % (r["ratio_median"], r["ratio_ci95_lo"], r["ratio_ci95_hi"]),
                                fp(r["p_raw"]), fp(r["p_holm_dataset"]), fp(r["p_holm_family"]), mark(r),
                                "R" if r["ci_hi_gt_regression_ratio"] else ""]
                    rows.append(row)
            L.append(md_table(["seq", "arm", "n_ok", "median", "mean", "std", "Δ median %", "ratio [95% CI]",
                               "p raw", "p Holm-D", "p Holm-F", "mark", "R"], rows,
                              ["l", "l", "r", "r", "r", "r", "r", "r", "r", "r", "r", "l", "l"]))
            L.append("")

    # (a)
    L.append("## (a) Significant comparisons vs expected by chance")
    L.append("")
    for metric in metrics:
        if metric not in S:
            continue
        L.append("**%s**" % mlabel(metric))
        L.append("")
        rows = []
        blocks = ([(arm, S[metric]["by_arm"][arm]) for arm in arms]
                  + [("non-null arms combined", S[metric]["non_null_arms_combined"]), ("ALL ARMS", S[metric]["overall"])])
        for name, b in blocks:
            rows.append([name, b["n_comparisons"],
                         "%d (%d better / %d worse)" % (b["raw_lt_alpha"], b["raw_lt_alpha_better"], b["raw_lt_alpha_worse"]),
                         "%.2f" % b["expected_raw_by_chance"],
                         "-" if b["expected_raw_at_null_control_rate"] is None else "%.2f" % b["expected_raw_at_null_control_rate"],
                         fp(b["binomial_P_X_ge_k_under_null"]),
                         "-" if b["P_at_least_one_raw_under_global_null"] is None else "%.3f" % b["P_at_least_one_raw_under_global_null"],
                         b["holm_dataset_lt_alpha"], b["holm_family_lt_alpha"],
                         "-" if b["holm_family_excl_null_lt_alpha"] is None else b["holm_family_excl_null_lt_alpha"]])
        L.append(md_table(["arm", "m", "raw p<%.2f" % P["alpha"], "expected by chance (0.05·m)",
                           "expected at null-control rate", "P(X≥k), X~Bin(m,0.05)", "P(≥1 raw hit under global null)",
                           "Holm-D p<%.2f" % P["alpha"], "Holm-F p<%.2f" % P["alpha"], "Holm-F excl. null arm"],
                          rows, ["l", "r", "l", "r", "r", "r", "r", "r", "r", "r"]))
        L.append("")
        if S[metric]["null_control_rate_used_for_expected"] is not None:
            L.append("`expected at null-control rate` = m x (raw hits of the `%s` arm / its comparisons) = m x %.4f; "
                     "`Holm-F excl. null arm` = Holm over the %d comparisons of the non-null arms only (sensitivity; "
                     "the main Holm-F keeps all %d)." % (
                         null_label, S[metric]["null_control_rate_used_for_expected"],
                         S[metric]["non_null_arms_combined"]["n_comparisons"], S[metric]["overall"]["n_comparisons"]))
            L.append("")
        rows = []
        for key, b in S[metric]["by_dataset_arm"].items():
            ds, arm = key.split("/")
            rows.append([ds, arm, b["n_comparisons"], b["raw_lt_alpha"], "%.2f" % b["expected_raw_by_chance"],
                         b["holm_dataset_lt_alpha"], b["holm_family_lt_alpha"]])
        L.append(md_table(["dataset", "arm", "m", "raw p<%.2f" % P["alpha"], "expected (0.05·m)",
                           "Holm-D", "Holm-F"], rows, ["l", "l", "r", "r", "r", "r", "r"]))
        L.append("")
    R = S.get("p_resolution")
    if R:
        L.append("**Resolution of the exact test.**")
        L.append("")
        for e in R["equal_size_complete_separation"]:
            if "p_complete_separation" in e:
                L.append("- n_arm = n_base = %d (%d comparisons): two completely separated samples give p = %d/%d = %.3g; "
                         "smallest raw p in the data: %s" % (
                             e["n_arm"], e["n_comparisons"], e["count_at_complete_separation"], e["total_relabelings"],
                             e["p_complete_separation"], ", ".join("%s %.3g (reached by %d comparison%s)" % (
                                 mlabel(m), v, R["n_comparisons_at_min_p_raw"][m],
                                 "" if R["n_comparisons_at_min_p_raw"][m] == 1 else "s")
                                 for m, v in R["min_p_raw_observed"].items())))
            else:
                L.append("- n_arm = %d, n_base = %d (%d comparisons): %s" % (e["n_arm"], e["n_base"], e["n_comparisons"], e.get("note", "")))
        for m, v in R["holm_first_rank_raw_p_threshold"].items():
            L.append("- Holm reaches alpha only if the raw p of the top-ranked comparison is <= alpha/m: %s family %.3g; by dataset %s" % (
                mlabel(m), v["family"], ", ".join("%s %.3g" % (ds, x) for ds, x in v["by_dataset"].items())))
        L.append("")
    for n in S["notes"]:
        L.append("- " + n)
    L.append("")

    # (b)
    if "b_null_control" in S:
        if flat:
            L.append("## (b) A/A check: `%s` vs `%s` (identical settings, run interleaved: these comparisons are the "
                     "empirical false-positive estimate)" % (null_label, base))
        else:
            L.append("## (b) Null control: `%s` vs `%s` (same trajectory by construction, per the project definition)" % (null_label, base))
        L.append("")
        for metric, nb in S["b_null_control"].items():
            L.append("**%s**" % mlabel(metric))
            L.append("")
            L.append("- raw p<%.2f: **%d of %d** comparisons (rate %.1f%%, Clopper-Pearson 95%% CI [%.1f%%, %.1f%%]); "
                     "expected at the nominal level: %.2f; P(X≥k | Bin(%d, 0.05)) = %.4f." % (
                         P["alpha"], nb["raw_lt_alpha"], nb["n_comparisons"], 100 * nb["false_positive_rate"],
                         100 * nb["clopper_pearson_95"][0], 100 * nb["clopper_pearson_95"][1],
                         nb["expected_by_chance"], nb["n_comparisons"], nb["binomial_P_X_ge_k_under_null"]))
            L.append("- by dataset: " + ", ".join("%s %d/%d" % (ds, v["raw_lt_alpha"], v["n"])
                                                   for ds, v in nb["by_dataset"].items()))
            L.append("- comparisons with raw p<%.2f: %s" % (P["alpha"], lst(nb["sequences_raw_lt_alpha"])))
            L.append("- Holm p<%.2f: family %d, dataset %d" % (P["alpha"], nb["holm_family_lt_alpha"], nb["holm_dataset_lt_alpha"]))
            L.append("- sorted raw p: " + ", ".join(fp(p) for p in nb["p_raw_sorted"]))
            L.append("- |change of median| %%: min %.1f, median %.1f, max %.1f; signed: min %+.1f, median %+.1f, max %+.1f; "
                     "direction: %d better, %d worse, %d tie" % (
                         nb["abs_rel_change_pct"]["min"], nb["abs_rel_change_pct"]["median"], nb["abs_rel_change_pct"]["max"],
                         nb["signed_rel_change_pct"]["min"], nb["signed_rel_change_pct"]["median"],
                         nb["signed_rel_change_pct"]["max"], nb["direction_counts"]["better"],
                         nb["direction_counts"]["worse"], nb["direction_counts"]["tie"]))
            L.append("- bootstrap ratio CI excludes 1.0 in %d of %d; CI upper bound > %.2f in %d" % (
                nb["ratio_ci_excludes_1"], nb["n_comparisons"], P["regression_ratio"], nb["ratio_ci_upper_gt_regression"]))
            L.append("")
            rows = [[r["dataset"] + "/" + r["sequence"], fpct(r["rel_change_median_pct"]),
                     "%.3f [%.3f, %.3f]" % (r["ratio_median"], r["ratio_ci95"][0], r["ratio_ci95"][1]), fp(r["p_raw"])]
                    for r in nb["per_comparison"]]
            L.append(md_table(["dataset/sequence", "Δ median %", "ratio [95% CI]", "p raw"], rows, ["l", "r", "r", "r"]))
            L.append("")

    # (c)
    L.append("## (c) Better / worse sequences per arm")
    L.append("")
    reg_key = "ratio_ci_upper_gt_%.2f" % P["regression_ratio"]
    for metric in metrics:
        if metric not in S:
            continue
        L.append("**%s**" % mlabel(metric))
        L.append("")
        for arm in arms:
            c = S[metric]["lists_c"][arm]
            L.append("- `%s`" % arm)
            L.append("  - better, raw p<%.2f: %s" % (P["alpha"], lst(c["better_raw"])))
            L.append("  - worse, raw p<%.2f: %s" % (P["alpha"], lst(c["worse_raw"])))
            L.append("  - better, Holm-F p<%.2f: %s" % (P["alpha"], lst(c["better_holm_family"])))
            L.append("  - worse, Holm-F p<%.2f: %s" % (P["alpha"], lst(c["worse_holm_family"])))
            L.append("  - better, Holm-D p<%.2f: %s" % (P["alpha"], lst(c["better_holm_dataset"])))
            L.append("  - worse, Holm-D p<%.2f: %s" % (P["alpha"], lst(c["worse_holm_dataset"])))
            L.append("  - ratio CI upper bound > %.2f: %s" % (P["regression_ratio"], "none" if not c[reg_key] else "; ".join(
                "%s/%s ratio %.3f [%.3f, %.3f]" % (i["dataset"], i["sequence"], i["ratio_median"], i["ratio_ci95"][0],
                                                   i["ratio_ci95"][1]) for i in c[reg_key])))
        L.append("")

    # (d)
    L.append("## (d) Wins / losses / ties by dataset")
    L.append("")
    L.append("`sign`: win = lower median than baseline, loss = higher, tie = equal. `raw`/`Holm-D`/`Holm-F`: win/loss only "
             "if the difference reaches p<%.2f at that level, otherwise tie (no detectable difference)." % P["alpha"])
    L.append("")
    for metric in metrics:
        if metric not in S:
            continue
        L.append("**%s**" % mlabel(metric))
        L.append("")
        rows = []
        for key, b in S[metric]["wins_losses_ties_d"].items():
            ds, arm = key.split("/")
            f = lambda x: "%d / %d / %d" % (x["wins"], x["losses"], x["ties"])
            rows.append([ds, arm, b["n"], f(b["by_median_sign"]), f(b["by_raw_p"]), f(b["by_holm_dataset"]),
                         f(b["by_holm_family"])])
        L.append(md_table(["dataset", "arm", "n", "sign W/L/T", "raw W/L/T", "Holm-D W/L/T", "Holm-F W/L/T"], rows,
                          ["l", "l", "r", "r", "r", "r", "r"]))
        L.append("")

    # extra paired comparisons
    if pair_recs:
        L.append("## Extra paired comparisons (`--pair REF:ARM`)")
        L.append("")
        L.append("ARM is compared with REF exactly like an arm with the baseline (statistic |median(ARM) - median(REF)|, "
                 "ratio = median(ARM)/median(REF), lower = better). Holm = over all pair x sequence comparisons of the "
                 "metric (one family per metric); they are not part of the families, summaries or flags above.")
        L.append("")
        PS = doc["summary"]["pairs"]
        for metric in metrics:
            sub = [r for r in pair_recs if r["metric"] == metric]
            if not sub:
                continue
            L.append("**%s**" % mlabel(metric))
            L.append("")
            rows = []
            for r in sub:
                m = "" if not r["sig_raw"] else {"better": "↓", "worse": "↑"}.get(r["direction"], "=") + " r" + (",H" if r["sig_holm_pairs"] else "")
                rows.append([r["pair"], "%s/%s" % (r["dataset"], r["sequence"]), r["n_base_ok"], fnum(r["base_median"]),
                             r["n_arm_ok"], fnum(r["arm_median"]), fpct(r["rel_change_median_pct"]),
                             "%.3f [%.3f, %.3f]" % (r["ratio_median"], r["ratio_ci95_lo"], r["ratio_ci95_hi"]),
                             fp(r["p_raw"]), fp(r["p_holm_pairs"]), m, "R" if r["ci_hi_gt_regression_ratio"] else ""])
            L.append(md_table(["REF:ARM", "seq", "n REF", "median REF", "n ARM", "median ARM", "Δ median %", "ratio [95% CI]",
                               "p raw", "p Holm-pairs", "mark", "R"], rows,
                              ["l", "l", "r", "r", "r", "r", "r", "r", "r", "r", "l", "l"]))
            L.append("")
            ps = PS[metric]
            L.append("%d comparisons: raw p<%.2f in %d (%d better / %d worse; expected by chance %.2f), Holm-pairs p<%.2f in %d." % (
                ps["n_comparisons"], P["alpha"], ps["raw_lt_alpha"], ps["raw_lt_alpha_better"], ps["raw_lt_alpha_worse"],
                ps["expected_raw_by_chance"], P["alpha"], ps["holm_pairs_lt_alpha"]))
            L.append("")

    # run-order diagnostics
    D = doc.get("run_order_diagnostics")
    if D:
        L.append("## Run-order diagnostics (are runs inside one block exchangeable?)")
        L.append("")
        L.append("Pooled over %d ATE groups (n_ok >= 4); null = shuffling the values inside each group, %d resamples "
                 "(seed %d)." % (D["n_groups"], D["resamples"], P["seed"]))
        L.append("")
        L.append("- mean Spearman(run index, ATE): %+.3f (null mean %+.3f), MC p = %.3f" % (
            D["mean_spearman_run_index_vs_value"], D["null_mean_spearman"], D["p_trend_mc"]))
        L.append("- mean lag-1 autocorrelation of ranks: %+.3f (null mean %+.3f), MC p = %.3f" % (
            D["mean_lag1_rank_autocorrelation"], D["null_mean_lag1"], D["p_lag1_mc"]))
        if D["mean_normalised_rank_of_run00"] is not None:
            L.append("- mean normalised rank of run00 within its group (0 = lowest ATE, 1 = highest; 0.5 expected): %.3f over %d groups"
                     % (D["mean_normalised_rank_of_run00"], D["n_groups_with_run00"]))
        L.append("- per arm (mean Spearman / mean lag-1): " + "; ".join(
            "%s %+.3f / %+.3f" % (a, v["mean_spearman"], v["mean_lag1"]) for a, v in D["by_arm"].items()))
        L.append("")

    # flags
    L.append("## Flags (data quality, nothing interpreted)")
    L.append("")
    fl = doc["flags"]
    order = ["failed_run", "ok_run_invalid_value", "null_control_hit", "n_ok_below_declared", "coverage_low", "coverage_differs_arm_vs_baseline",
             "coverage_varies_within_group", "outliers", "heavy_spread", "exact_ties", "pooled_exact_ties",
             "permutation_tie_mass", "walltime_outlier", "aggregate_mismatch", "monte_carlo_p", "small_n"]
    titles = {"failed_run": "Failed runs (dropped)", "ok_run_invalid_value": "Runs with status ok but an invalid value",
              "null_control_hit": "Null-control arm differs from baseline at raw p<alpha", "n_ok_below_declared": "Groups with n_ok below the declared run count",
              "coverage_low": "Very low coverage (ATE computed on few poses)",
              "coverage_differs_arm_vs_baseline": "Median coverage differs between arm and baseline",
              "coverage_varies_within_group": "Coverage varies between runs of one group",
              "outliers": "ATE outliers (Tukey 1.5xIQR fences; per group)", "heavy_spread": "Heavy spread within a group",
              "exact_ties": "Exact duplicate values", "pooled_exact_ties": "Exact ties in the pooled sample",
              "permutation_tie_mass": "Permutation distribution tie mass at the observed statistic",
              "walltime_outlier": "Wall-time outliers", "aggregate_mismatch": "Disagreement with results.json aggregates",
              "monte_carlo_p": "Monte Carlo p-values", "small_n": "Sample size"}
    seen_types = {f["type"] for f in fl}
    for t in order:
        items = [f for f in fl if f["type"] == t]
        if not items:
            continue
        L.append("### %s (%d)" % (titles[t], len(items)))
        L.append("")
        for f in items:
            extra = ""
            if f["type"] == "failed_run" and f["data"].get("log_tail") is not None:
                extra = " | last log line: " + (f["data"]["log_tail"][-1][:160] if f["data"]["log_tail"] else "(empty log)")
            L.append("- `%s`: %s%s" % (f["where"], f["detail"], extra))
        L.append("")
    for t in sorted(seen_types - set(order)):
        L.append("### %s" % t)
        for f in fl:
            if f["type"] == t:
                L.append("- `%s`: %s" % (f["where"], f["detail"]))
        L.append("")
    # explicit negatives, so absence is visible
    absent = [titles[t] for t in ("exact_ties", "pooled_exact_ties", "aggregate_mismatch", "monte_carlo_p") if t not in seen_types]
    if absent:
        L.append("Not found: " + "; ".join(absent) + ".")
        L.append("")
    L.append("## Reproduce")
    L.append("")
    L.append("`nice -n 19 python3 evaluation/harness/stats_compare.py --results %s --out <dir>` (deterministic; "
             "script sha256 %s)." % (doc["meta"]["results_dir"], doc["meta"]["script_sha256"][:16]))
    L.append("")
    return "\n".join(L)


# --------------------------------------------------------------------------- driver

def clean(obj):
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [clean(v) for v in obj]
    return obj


def parse_pairs(specs):
    """['REF:ARM,REF2:ARM2', ...] -> [(REF, ARM), ...]"""
    out = []
    for spec in specs or []:
        for part in spec.split(","):
            if not part:
                continue
            ab = part.split(":")
            if len(ab) != 2 or not ab[0] or not ab[1] or ab[0] == ab[1]:
                sys.exit("--pair needs REF:ARM with two different arm names, got %r" % part)
            if (ab[0], ab[1]) not in out:
                out.append((ab[0], ab[1]))
    return out


def resolve_layout(args, results_dir):
    if args.layout != "auto":
        return args.layout
    return "flat" if os.path.isfile(os.path.join(results_dir, args.baseline, "results.json")) else "legacy"


def arm_dir(cfg, ds, arm):
    """How the arm's results directory is called in this layout (for messages)."""
    return arm if cfg["layout"] == "flat" else "%s_%s" % (ds, arm)


def analyse(args):
    t0 = time.time()
    results_dir = os.path.abspath(args.results)
    layout = resolve_layout(args, results_dir)
    metrics = [m for m in args.metrics.split(",") if m]
    if not metrics:
        sys.exit("--metrics is empty")
    pairs = parse_pairs(args.pair)
    pair_arms = [a for pr in pairs for a in pr]
    arm_filter = args.arms.split(",") if args.arms else None
    if layout == "flat":
        found, datasets, arms_by_ds = discover_flat(results_dir, args.baseline, arm_filter, pair_arms)
    else:
        found, datasets, arms_by_ds = discover(results_dir, args.baseline,
                                               args.datasets.split(",") if args.datasets else None, arm_filter)
        if not datasets:
            sys.exit("no <dataset>_%s/results.json found under %s" % (args.baseline, results_dir))
    arms = []
    for ds in datasets:
        for a in arms_by_ds[ds]:
            if a not in arms:
                arms.append(a)
    if args.null_arms is not None:
        null_arms = [a for a in args.null_arms.split(",") if a]
        bad = [a for a in null_arms if a not in arms]
        if bad:
            sys.exit("--null-arms %s: not among the arms compared with the baseline (%s)" % (", ".join(bad), ", ".join(arms)))
    elif args.null_arm is not None:
        null_arms = [args.null_arm]
    else:
        null_arms = ["offline"] if layout == "legacy" else []
    load_by_ds = {ds: [args.baseline] + arms_by_ds[ds] + [a for a in dict.fromkeys(pair_arms)
                                                           if a != args.baseline and a not in arms_by_ds[ds]
                                                           and (ds, a) in found] for ds in datasets}
    docs, inputs = {}, []
    for ds in datasets:
        for arm in load_by_ds[ds]:
            p = found[(ds, arm)]
            with open(p, encoding="utf-8") as f:
                docs[(ds, arm)] = json.load(f)
            inputs.append({"dataset": ds, "arm": arm, "path": p, "sha256": _sha256(p), "bytes": os.path.getsize(p)})
    cfg = {"exact_max": args.perm_exact_max, "mc": args.perm_mc, "seed": args.seed, "boot": args.boot,
           "alpha": args.alpha, "regression_ratio": args.regression_ratio, "heavy_spread": args.heavy_spread,
           "coverage_tol": args.coverage_tol, "walltime_ratio": args.walltime_ratio, "null_arm": null_arms[0] if null_arms else None,
           "null_arms": null_arms, "metrics": metrics, "layout": layout}
    groups, seq_by_ds, extra_flags = {}, {}, []
    for ds in datasets:
        bdoc = docs[(ds, args.baseline)]
        seqs = list(bdoc["sequences"].keys())
        seq_by_ds[ds] = seqs
        for arm in load_by_ds[ds]:
            doc = docs[(ds, arm)]
            where = arm_dir(cfg, ds, arm)
            for seq in seqs:
                if seq not in doc["sequences"]:
                    extra_flags.append({"type": "missing_sequence", "severity": "warn", "where": f"{where}/{seq}",
                                        "detail": "sequence absent from this arm's results.json", "data": {}})
                    continue
                for metric in metrics:
                    if metric != "ate_rmse" and not any(_finite(get_path(r, metric)) for r in doc["sequences"][seq]["runs"]):
                        continue
                    groups[(ds, arm, seq, metric)] = make_group(ds, arm, seq, metric, doc, results_dir)
            for seq in doc["sequences"]:
                if seq not in seqs:
                    extra_flags.append({"type": "extra_sequence", "severity": "info", "where": f"{where}/{seq}",
                                        "detail": "sequence not in the baseline; not compared", "data": {}})
    recs = run_comparisons(groups, datasets, arms_by_ds, seq_by_ds, args.baseline, cfg, args.jobs)
    pair_recs = run_pair_comparisons(groups, datasets, seq_by_ds, pairs, cfg, args.jobs)
    summ = summaries(recs, datasets, arms, null_arms, cfg)
    if pairs:
        summ["pairs"] = pair_summary(pair_recs, pairs, cfg)
    flags = build_flags(groups, recs, datasets, arms_by_ds, args.baseline, cfg) + extra_flags
    diag = run_order_diagnostics(groups, args.seed, args.diag_resamples)
    script = os.path.abspath(__file__)
    families = {"ate_rmse_holm_family": sum(1 for r in recs if r["metric"] == "ate_rmse"),
                "t_rel_pct_holm_family": sum(1 for r in recs if r["metric"] == "t_rel_pct")}
    for m in metrics:
        if m not in METRICS:
            families["%s_holm_family" % m] = sum(1 for r in recs if r["metric"] == m)
    if pairs:
        families["pairs_holm_family_per_metric"] = {m: sum(1 for r in pair_recs if r["metric"] == m) for m in metrics}
    doc = {"meta": {
        "schema_version": SCHEMA_VERSION,
        "created_utc": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%d %H:%M:%S"),
        "elapsed_seconds": round(time.time() - t0, 1),
        "script": script, "script_sha256": _sha256(script),
        "python": sys.version.split()[0], "platform": platform.platform(),
        "results_dir": results_dir, "baseline_arm": args.baseline, "null_arm": cfg["null_arm"],
        "datasets": datasets, "arms": arms, "arms_by_dataset": arms_by_ds, "sequences": seq_by_ds,
        "parameters": {"alpha": args.alpha, "seed": args.seed, "perm_exact_max": args.perm_exact_max,
                       "perm_mc_shuffles": args.perm_mc, "perm_tolerance": TOL, "bootstrap_resamples": args.boot,
                       "bootstrap_level": 0.95, "regression_ratio": args.regression_ratio,
                       "heavy_spread_max_over_min": args.heavy_spread, "coverage_tolerance": args.coverage_tol,
                       "walltime_outlier_ratio": args.walltime_ratio},
        "families": families,
        "inputs": inputs,
        "layout": layout, "null_arms": null_arms, "pairs": ["%s:%s" % pr for pr in pairs], "metrics": metrics},
        # insertion order = dataset, arm (baseline first), sequence, metric
        "groups": list(groups.values()),
        "comparisons": recs, "pair_comparisons": pair_recs, "summary": summ, "run_order_diagnostics": diag,
        "flags": flags}
    return clean(doc)


def run_self_test():
    import statistics as st
    failures = []

    def check(name, cond, info=""):
        print(("PASS " if cond else "FAIL ") + name + (("  " + info) if info else ""))
        if not cond:
            failures.append(name)

    adj = holm_adjust([0.01, 0.04, 0.03, 0.005])
    check("holm vs R p.adjust(c(.01,.04,.03,.005),'holm') = .03 .06 .06 .02",
          all(math.isclose(a, b, abs_tol=1e-12) for a, b in zip(adj, [0.03, 0.06, 0.06, 0.02])), str(adj))
    check("holm caps at 1", holm_adjust([0.4, 0.4, 0.9]) == [1.0, 1.0, 1.0], str(holm_adjust([0.4, 0.4, 0.9])))
    adj = holm_adjust([0.01, 0.5, 0.01])
    check("holm gives tied p-values equal adjusted values (R: .03 .5 .03)",
          all(math.isclose(a, b, abs_tol=1e-12) for a, b in zip(adj, [0.03, 0.5, 0.03])), str(adj))
    check("quantile_linear", quantile_linear([1, 2, 3, 4], 0.5) == 2.5 and quantile_linear([1, 2, 3, 4], 0.25) == 1.75)
    check("median even/odd", median([3, 1, 2]) == 2 and median([4, 1, 2, 3]) == 2.5)
    check("sample std ddof=1", math.isclose(stdev([1, 2, 3, 4]), st.stdev([1, 2, 3, 4])))

    def brute(arm, base):
        pooled, n1, n = list(arm) + list(base), len(arm), len(arm) + len(base)
        obs = abs(st.median(arm) - st.median(base))
        cnt = tot = 0
        for mask in range(1 << n):
            if bin(mask).count("1") != n1:
                continue
            a = [pooled[i] for i in range(n) if mask >> i & 1]
            b = [pooled[i] for i in range(n) if not mask >> i & 1]
            tot += 1
            cnt += abs(st.median(a) - st.median(b)) >= obs - TOL
        return cnt, tot

    rnd = random.Random(1)
    ok = True
    for n1, n2 in ((3, 3), (4, 5), (5, 4), (6, 6), (2, 7), (5, 5)):
        for trial in range(6):
            arm = [round(rnd.gauss(0, 1), 1) for _ in range(n1)]      # rounding creates ties on purpose
            base = [round(rnd.gauss(0.3 * trial, 1), 1) for _ in range(n2)]
            r = permutation_test(arm, base)
            ok &= (r["count_ge"], r["total"]) == brute(arm, base) and r["method"] == "exact"
    check("exact permutation == brute force (incl. ties, unequal n)", ok)
    check("C(20,10) relabelings -> exact", permutation_test([1.0] * 10, [2.0] * 10)["total"] == 184756)
    sf = separation_floor(10)
    check("two completely separated generic samples of 10: p = 140/184756 (both orientations)",
          sf["count_ge"] == 140 and sf["total"] == 184756 and
          permutation_test(sorted([1000.0 + math.log(q) for q in _first_primes(20)[10:]]),
                           [math.log(q) for q in _first_primes(10)])["count_ge"] == 140, "%d/%d" % (sf["count_ge"], sf["total"]))
    a = [0.11, 0.35, 0.12, 0.52, 0.19, 0.28, 0.44, 0.15]
    b = [0.25, 0.22, 0.41, 0.33, 0.61, 0.39, 0.47, 0.30]
    pe = permutation_test(a, b)
    pm = permutation_test(a, b, exact_max=1, mc_shuffles=200000, seed=12345)
    check("Monte Carlo branch agrees with exact (|dp| < 0.01)", pm["method"] == "monte_carlo" and abs(pe["p"] - pm["p"]) < 0.01,
          "exact %.4f mc %.4f" % (pe["p"], pm["p"]))
    check("Monte Carlo is reproducible for a fixed seed",
          permutation_test(a, b, exact_max=1, mc_shuffles=5000, seed=7)["p"] == permutation_test(a, b, exact_max=1, mc_shuffles=5000, seed=7)["p"])
    lo, hi, n = bootstrap_ratio_ci(a, a, 2000, 12345)
    check("bootstrap CI of identical samples brackets 1", lo < 1.0 < hi, "[%.3f, %.3f]" % (lo, hi))
    check("bootstrap reproducible", bootstrap_ratio_ci(a, b, 500, 12345) == bootstrap_ratio_ci(a, b, 500, 12345))
    lo, up = clopper_pearson(0, 17)
    check("Clopper-Pearson k=0: upper = 1-(alpha/2)^(1/n)", lo == 0.0 and math.isclose(up, 1 - 0.025 ** (1 / 17), rel_tol=1e-9), "%.4f" % up)
    lo, up = clopper_pearson(4, 17)
    check("Clopper-Pearson k=4,n=17 brackets the estimate and inverts the tails",
          lo < 4 / 17 < up and math.isclose(binom_sf(4, 17, lo), 0.025, rel_tol=1e-6) and math.isclose(binom_cdf(4, 17, up), 0.025, rel_tol=1e-6))
    check("binomial tail", math.isclose(binom_sf(1, 17, 0.05), 1 - 0.95 ** 17, rel_tol=1e-12))
    self_test_flat(check)
    print("\nself-test: %s" % ("ALL PASSED" if not failures else "FAILED: " + ", ".join(failures)))
    return 0 if not failures else 1


def self_test_flat(check):
    """Flat layout, A/A arms, pairs and extra metrics on synthetic results (no real data needed)."""
    import tempfile
    check("get_path: dotted path, plain key first, None when absent",
          get_path({"a": {"b": 3}}, "a.b") == 3 and get_path({"a.b": 1, "a": {"b": 2}}, "a.b") == 1 and get_path({"a": 1}, "a.b") is None
          and get_path({"x": None}, "x") is None)
    check("dataset label: meta.dataset, else the config's first word, else 'all'",
          _dataset_label({"meta": {"dataset": "kitti"}}, "x") == "kitti" and _dataset_label({"config": "euroc_stereo"}, "x") == "euroc"
          and _dataset_label({"config": "weird"}, "x") == "all")
    check("parse_pairs: repeatable and comma lists, de-duplicated", parse_pairs(["a:b,c:d", "a:b"]) == [("a", "b"), ("c", "d")])
    rnd = random.Random(7)

    def write_arm(root, name, mu, n=8, dataset="euroc", seqs=("S1", "S2")):
        os.makedirs(os.path.join(root, name))
        doc = {"config": name, "tag": name, "runs": n, "meta": {"dataset": dataset}, "sequences": {}}
        for sq in seqs:
            runs = []
            for i in range(n):
                v = mu * (1.0 + 0.04 * rnd.gauss(0, 1))
                runs.append({"run": "run%02d" % i, "status": "ok", "returncode": 0, "seconds": 100.0, "ate_rmse": v, "coverage": 1.0,
                             "scores": {"kf_se3": 1.2 * v}})
            doc["sequences"][sq] = {"runs": runs, "failures": 0}
        with open(os.path.join(root, name, "results.json"), "w") as fh:
            json.dump(doc, fh)

    with tempfile.TemporaryDirectory() as d:
        for name, mu in (("base", 1.0), ("base_b", 1.0), ("c1", 0.7), ("imp", 0.6)):
            write_arm(d, name, mu)
        write_arm(d, "other_ds", 1.0, dataset="kitti")
        argv = ["--results", d, "--out", d, "--baseline", "base", "--arms", "base_b,c1,imp", "--null-arms", "base_b",
                "--pair", "c1:imp", "--boot", "300", "--diag-resamples", "0", "--metrics", "ate_rmse,scores.kf_se3"]
        doc = analyse(make_parser().parse_args(argv))
        mt = doc["meta"]
        check("flat layout is detected from <results>/<baseline>/results.json",
              mt["layout"] == "flat" and mt["datasets"] == ["euroc"] and mt["arms"] == ["base_b", "c1", "imp"] and mt["baseline_arm"] == "base")
        check("3 arms x 2 sequences x 2 metrics = 12 arm-vs-baseline comparisons, none of them a pair",
              len(doc["comparisons"]) == 12 and all("pair" not in r for r in doc["comparisons"]))
        c1 = [r for r in doc["comparisons"] if r["arm"] == "c1" and r["metric"] == "ate_rmse"]
        check("a 30% lower arm is significant and 'better'; Holm adjusted values exist",
              len(c1) == 2 and all(r["sig_raw"] and r["direction"] == "better" and r["p_holm_family"] is not None for r in c1))
        aa = [r for r in doc["comparisons"] if r["arm"] == "base_b"]
        check("the A/A replicate is not significant (same distribution)", len(aa) == 4 and not any(r["sig_raw"] for r in aa),
              "p %s" % ", ".join("%.3f" % r["p_raw"] for r in aa))
        nb = doc["summary"]["b_null_control"]["ate_rmse"]
        check("A/A comparisons are reported as the false-positive estimate", nb["arms"] == ["base_b"] and nb["n_comparisons"] == 2
              and nb["raw_lt_alpha"] == 0 and nb["false_positive_rate"] == 0.0 and len(nb["clopper_pearson_95"]) == 2)
        check("null arm is excluded from the non-null combination and from the 'excl. null' Holm family",
              doc["summary"]["ate_rmse"]["non_null_arms_combined"]["n_comparisons"] == 4
              and all(r["p_holm_family_excl_null"] is None for r in aa) and all(r["family_size_excl_null"] == 4 for r in c1))
        pr = doc["pair_comparisons"]
        check("pair c1:imp: imp against c1, 2 sequences x 2 metrics, own Holm family",
              len(pr) == 4 and all(r["pair"] == "c1:imp" and r["arm"] == "imp" and r["baseline_arm"] == "c1" for r in pr)
              and all(r["pair_family_size"] == 2 and r["p_holm_pairs"] >= r["p_raw"] for r in pr) and all(r["direction"] == "better" for r in pr))
        check("pair ratio is median(imp)/median(c1)", all(abs(r["ratio_median"] - r["arm_median"] / r["base_median"]) < 1e-12 for r in pr))
        check("extra metric (scores.kf_se3) is its own family", doc["meta"]["families"]["scores.kf_se3_holm_family"] == 6
              and doc["summary"]["scores.kf_se3"]["overall"]["n_comparisons"] == 6)
        md = render_markdown(doc)
        check("markdown has the A/A section, the pair section and the extra-metric tables",
              "A/A check" in md and "Extra paired comparisons" in md and "scores.kf_se3, euroc" in md and "c1:imp" in md)
        check("legacy-only wording is absent in flat mode", "same trajectory by construction" not in md)

        def exits(extra, needle):
            base_argv = [a for i, a in enumerate(argv) if a != "--arms" and (i == 0 or argv[i - 1] != "--arms")]
            try:
                analyse(make_parser().parse_args(base_argv + extra))
            except SystemExit as exc:
                return needle in str(exc.code)
            return False

        check("flat layout refuses an arm of another dataset", exits(["--arms", "base_b,other_ds"], "one dataset"))
        check("--null-arms must name a compared arm", exits(["--arms", "c1"], "not among the arms"))
        check("a missing baseline is an error that names what exists", exits(["--baseline", "nope", "--arms", "c1"], "nope/results.json"))

    with tempfile.TemporaryDirectory() as d:
        for name, mu in (("tum_baseline", 1.0), ("tum_offline", 1.0), ("tum_dense_imp", 0.8)):
            write_arm(d, name, mu, dataset="tum")
        for name in ("tum_baseline", "tum_offline", "tum_dense_imp"):  # legacy dirs carry no meta.dataset
            with open(os.path.join(d, name, "results.json")) as fh:
                doc = json.load(fh)
            doc.pop("meta")
            with open(os.path.join(d, name, "results.json"), "w") as fh:
                json.dump(doc, fh)
        doc = analyse(make_parser().parse_args(["--results", d, "--out", d, "--boot", "200", "--diag-resamples", "0"]))
        check("legacy layout is still the default when <results>/baseline/ does not exist",
              doc["meta"]["layout"] == "legacy" and doc["meta"]["datasets"] == ["tum"] and doc["meta"]["null_arms"] == ["offline"]
              and doc["meta"]["arms"] == ["dense_imp", "offline"] and doc["meta"]["null_arm"] == "offline")
        check("legacy null control keeps its single-arm shape", doc["summary"]["b_null_control"]["ate_rmse"]["arm"] == "offline"
              and doc["pair_comparisons"] == [])
        md = render_markdown(doc)
        check("legacy wording is unchanged", "Null control: `offline` vs `baseline` (same trajectory by construction" in md and "A/A check" not in md)
        doc = analyse(make_parser().parse_args(["--results", d, "--out", d, "--boot", "200", "--diag-resamples", "0", "--pair", "offline:dense_imp"]))
        check("pairs work in the legacy layout too (per dataset where both arms exist)",
              len(doc["pair_comparisons"]) == 2 and doc["pair_comparisons"][0]["baseline_arm"] == "offline")


def make_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results", help="directory holding <dataset>_<arm>/results.json (flat layout: <arm>/results.json)")
    ap.add_argument("--out", help="output directory (created); writes stats_all.json and stats_table.md")
    ap.add_argument("--layout", choices=["auto", "legacy", "flat"], default="auto",
                    help="legacy = <dataset>_<arm>/results.json; flat = <arm>/results.json with free arm names; "
                         "auto = flat when <results>/<baseline>/results.json exists (default)")
    ap.add_argument("--baseline", default="baseline",
                    help="name of the reference arm (default: baseline; flat layout: its directory name)")
    ap.add_argument("--null-arm", default=None,
                    help="one arm whose trajectory equals the baseline's by construction (legacy default: offline)")
    ap.add_argument("--null-arms", default=None,
                    help="comma list of arms that must not differ from the baseline (A/A replicates, or offline); "
                         "their comparisons are reported as the empirical false-positive estimate")
    ap.add_argument("--pair", action="append", default=[], metavar="REF:ARM",
                    help="extra comparison of ARM against REF instead of the baseline (repeatable or comma list); "
                         "own Holm family, own section")
    ap.add_argument("--metrics", default=",".join(METRICS),
                    help="comma list of per-run metrics (dotted path in the run entry, e.g. scores.kf_se3); every one "
                         "is a cost, lower = better, and its own Holm family (default: %s)" % ",".join(METRICS))
    ap.add_argument("--datasets", help="comma list to restrict datasets (default: all found; legacy layout)")
    ap.add_argument("--arms", help="comma list to restrict arms (default: all found); flat layout: the arms compared "
                                   "with the baseline")
    ap.add_argument("--seed", type=int, default=12345)
    ap.add_argument("--boot", type=int, default=10000, help="bootstrap resamples")
    ap.add_argument("--perm-exact-max", type=int, default=400000, help="max C(n1+n2,n1) for exact enumeration")
    ap.add_argument("--perm-mc", type=int, default=200000, help="Monte Carlo shuffles when not exact")
    ap.add_argument("--alpha", type=float, default=0.05)
    ap.add_argument("--regression-ratio", type=float, default=1.10, help="flag ratio CI upper bound above this")
    ap.add_argument("--heavy-spread", type=float, default=3.0, help="flag groups with max/min at least this")
    ap.add_argument("--coverage-tol", type=float, default=0.005, help="coverage differences above this are flagged")
    ap.add_argument("--walltime-ratio", type=float, default=1.5, help="flag runs slower than this x group median")
    ap.add_argument("--diag-resamples", type=int, default=2000, help="MC resamples for the run-order diagnostics (0 = skip)")
    ap.add_argument("--jobs", type=int, default=1, help="worker processes (does not change any result)")
    ap.add_argument("--self-test", action="store_true", help="run the built-in checks and exit")
    return ap


def main():
    ap = make_parser()
    args = ap.parse_args()
    if args.self_test:
        sys.exit(run_self_test())
    if not args.results or not args.out:
        ap.error("--results and --out are required")
    doc = analyse(args)
    os.makedirs(args.out, exist_ok=True)
    jpath, mpath = os.path.join(args.out, "stats_all.json"), os.path.join(args.out, "stats_table.md")
    with open(jpath, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=1, allow_nan=False)
    with open(mpath, "w", encoding="utf-8") as f:
        f.write(render_markdown(doc))
    first = doc["meta"]["metrics"][0]
    S = doc["summary"].get(first)
    print("wrote %s and %s (%.1f s)" % (jpath, mpath, doc["meta"]["elapsed_seconds"]))
    if S:
        print("%s comparisons: %d | raw p<%.2f: %d (expected %.2f) | Holm-F: %d | Holm-D: %d" % (
            "ATE" if first == "ate_rmse" else first, S["overall"]["n_comparisons"], args.alpha, S["overall"]["raw_lt_alpha"],
            S["overall"]["expected_raw_by_chance"], S["overall"]["holm_family_lt_alpha"], S["overall"]["holm_dataset_lt_alpha"]))
    nb = doc["summary"].get("b_null_control", {}).get(first)
    if nb:
        print("%s (%s): %d of %d raw p<%.2f" % ("A/A check" if doc["meta"]["layout"] == "flat" else "null control", nb["arm"],
                                               nb["raw_lt_alpha"], nb["n_comparisons"], args.alpha))
    for metric, ps in doc["summary"].get("pairs", {}).items():
        print("pairs, %s: %d comparisons | raw p<%.2f: %d (%d better / %d worse) | Holm-pairs: %d" % (
            metric, ps["n_comparisons"], args.alpha, ps["raw_lt_alpha"], ps["raw_lt_alpha_better"], ps["raw_lt_alpha_worse"],
            ps["holm_pairs_lt_alpha"]))


if __name__ == "__main__":
    main()
