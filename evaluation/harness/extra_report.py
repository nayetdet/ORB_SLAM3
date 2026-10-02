#!/usr/bin/env python3
"""Readable tables for the two extra experiments (evaluation/run_extra.sh). Standard library only, no numpy.

    python3 evaluation/harness/extra_report.py fps   --results DIR     # unpaced throughput, one dataset block
    python3 evaluation/harness/extra_report.py multi --results DIR     # multi-sequence chains, one dataset block

DIR holds <arm>/results.json written by run_interleaved.py. The tables are descriptive (medians, ranges, counts); the tests,
ratios with confidence intervals and the Holm correction are in DIR/stats (stats_compare.py).

fps    fps = the "Processed N frames in X s = Y fps" line: N frames / wall-clock time of the frame loop (image loading
       included, Shutdown excluded), ORB_NO_PACING=1 (no sleeping to the dataset timestamps). Higher is better; the ratio column
       is the arm's median fps over the baseline's.
multi  per chain and arm: runs ok / total (INVALID = the sequences did not merge into one map, chain_eval.py), ATE of the LAST
       sequence of the chain (Table VII's number), its mean error per frame, ATE of the whole chain under one alignment (small
       only if the merged map is consistent), maps in the atlas, merges detected. delta = last-sequence ATE of the chain relative
       to the single-sequence CONTROL of the same arm in the same session (negative = the chain helps).
"""

import argparse
import json
import os
import statistics
import sys

ORDER = ["base", "c1", "faithful", "imp"]


def load(results):
    arms = {}
    for name in sorted(os.listdir(results)):
        p = os.path.join(results, name, "results.json")
        if os.path.isfile(p):
            with open(p) as fh:
                arms[name] = json.load(fh)
    def key(a):
        suf = a.rsplit("_", 1)[-1]
        return (ORDER.index(suf) if suf in ORDER else 99, a)
    return [(a, arms[a]) for a in sorted(arms, key=key)]


def med(v):
    v = [x for x in v if x is not None]
    return statistics.median(v) if v else None


def f(x, d=4):
    return "—" if x is None else ("%.*f" % (d, x))


def perm_p(a, b, exact_max=200000):
    """Two-sided exact permutation test on the difference of medians (a, b: lists of numbers); None if too few or too many."""
    import itertools
    import math
    n1, n = len(a), len(a) + len(b)
    if n1 < 2 or len(b) < 2 or math.comb(n, n1) > exact_max:
        return None
    allv = list(a) + list(b)
    obs = abs(statistics.median(a) - statistics.median(b))
    hit = total = 0
    for idx in itertools.combinations(range(n), n1):
        s1 = set(idx)
        x = [allv[i] for i in idx]
        y = [allv[i] for i in range(n) if i not in s1]
        total += 1
        if abs(statistics.median(x) - statistics.median(y)) >= obs - 1e-12:
            hit += 1
    return hit / total


def get(d, path):
    for k in path.split("."):
        if not isinstance(d, dict):
            return None
        d = d.get(k)
    return d


def seq_names(arms):
    out = []
    for _, doc in arms:
        for s in doc["sequences"]:
            if s not in out:
                out.append(s)
    return out


def report_fps(arms):
    L = ["# Unpaced throughput (FPS)", "",
         "fps = N frames / wall-clock time of the frame loop, image loading included, Shutdown excluded, `ORB_NO_PACING=1`. "
         "Median over ok runs; range in brackets; ratio = median fps / baseline median fps.", ""]
    base_arm = arms[0][0]
    for seq in seq_names(arms):
        L += ["## %s" % seq, "", "| arm | ok/N | fps median [min, max] | ms/frame | ratio vs %s | wall s | ATE m | frames |" % base_arm,
              "|---|---|---|---|---|---|---|---|"]
        base = None
        for arm, doc in arms:
            runs = doc["sequences"].get(seq, {}).get("runs", [])
            ok = [r for r in runs if r.get("status") == "ok"]
            fps = [get(r, "log_metrics.fps") for r in ok]
            fps = [x for x in fps if x is not None]
            ft = med([get(r, "log_metrics.frame_time_ms") for r in ok])
            m = med(fps)
            if base is None:
                base = m
            ratio = (m / base) if (m is not None and base) else None
            nfr = med([get(r, "log_metrics.fps_frames") for r in ok])
            unp = sum(1 for r in ok if get(r, "log_metrics.fps_unpaced"))
            note = "" if unp == len(ok) else " (%d of %d runs without the ORB_NO_PACING notice!)" % (len(ok) - unp, len(ok))
            L.append("| %s | %d/%d | %s [%s, %s] | %s | %s | %s | %s | %s%s |" % (
                arm, len(ok), len(runs), f(m, 2), f(min(fps) if fps else None, 2), f(max(fps) if fps else None, 2),
                f(ft, 2), f(ratio, 3), f(med([r.get("seconds") for r in ok]), 0),
                f(med([r.get("ate_rmse") for r in ok]), 4), f(nfr, 0), note))
        L.append("")
    L += ["Thesis, for context only (different laptop; paced or not is not stated): Table IV FR1_desk 23.7 Hz, FR1_room 24.2, "
          "FR2_desk 19.6, FR2_no_loop 17.7; stereo EuRoC about 15 Hz at 1000 features; KITTI as low as 10 Hz (its input rate).", ""]
    return L


def report_multi(arms, reference=None):
    L = ["# Multi-sequence chains", "",
         "One process per chain run; ORB-SLAM3 merges the maps itself. ok = valid (one map in the atlas, every sequence >= 50% in "
         "the saved trajectory); INVALID runs are excluded from every median and listed below. ATE = RMSE after SE(3) alignment, m.", ""]
    ref = reference or {}
    t7, t5 = ref.get("thesis_table_vii", {}), ref.get("thesis_table_v_single", {})
    invalid = []
    for arm, doc in arms:
        L += ["## %s" % arm, "",
              "| chain | ok/N | last-seq ATE | delta vs control (p) | last-seq mean err | chain ATE (1 alignment) | last in chain align. | maps | merges | thesis VII (V) |",
              "|---|---|---|---|---|---|---|---|---|---|"]
        seqs = doc["sequences"]
        for name, s in seqs.items():
            runs = s.get("runs", [])
            ok = [r for r in runs if r.get("status") == "ok"]
            bad = [r for r in runs if r.get("status") == "invalid_chain"]
            for r in bad:
                invalid.append((arm, name, r.get("run"), r.get("error", "")))
            names = None
            for r in runs:
                names = names or (r.get("chain") or {}).get("names")
            last = med([r.get("ate_rmse") for r in ok])
            ctrl = None
            if names and len(names) > 1:
                cs = seqs.get(names[-1], {}).get("runs", [])
                ctrl = med([r.get("ate_rmse") for r in cs if r.get("status") == "ok"])
            delta = "—" if (last is None or not ctrl) else "%+.1f%%" % (100.0 * (last - ctrl) / ctrl)
            if names and len(names) > 1 and delta != "—":
                pv = perm_p([r["ate_rmse"] for r in ok], [r["ate_rmse"] for r in cs if r.get("status") == "ok"])
                delta += " (p=%s)" % ("n/a" if pv is None else "%.3f" % pv)
            maps = sorted({get(r, "chain.maps_in_atlas") for r in runs if get(r, "chain.maps_in_atlas") is not None})
            th = ""
            if name in t7:
                th = "%.3f (%.3f)" % (t7[name], t5.get(names[-1], float("nan"))) if names else "%.3f" % t7[name]
            elif name in t5:
                th = "(%.3f)" % t5[name]
            L.append("| %s | %d/%d | %s | %s | %s | %s | %s | %s | %s | %s |" % (
                name, len(ok), len(runs), f(last), delta, f(med([get(r, "scores.last_mean_m") for r in ok])),
                f(med([get(r, "scores.chain_se3") for r in ok])), f(med([get(r, "scores.last_in_chain_se3") for r in ok])),
                ",".join(str(m) for m in maps) or "—", f(med([get(r, "log_metrics.merges_detected") for r in runs]), 1), th))
        L.append("")
    if invalid:
        L += ["## INVALID chain runs (excluded)", "", "| arm | chain | run | reason |", "|---|---|---|---|"]
        for a, n, r, e in invalid:
            L.append("| %s | %s | %s | %s |" % (a, n, r, e.replace("INVALID CHAIN: ", "")))
        L.append("")
    else:
        L += ["No INVALID chain runs.", ""]
    L += ["Controls are the single sequences (a chain of one) of the same arm and session; delta = (chain - control) / control on "
          "medians; p = exact two-sided permutation test on the median difference, UNADJUSTED (several chains x arms are tested: "
          "multiply by the number of chains for a Bonferroni bound). Thesis columns: Table VII, and in brackets Table V (single sequence) for the same last sequence. TUM: the thesis "
          "gives the AVERAGE ATE PER FRAME of fr2_large_no_loop, 0.706 m single -> 0.181 m after fr2_large_with_loop: compare it "
          "with the 'last-seq mean err' column.", ""]
    return L


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kind", choices=["fps", "multi"])
    ap.add_argument("--results", required=True)
    ap.add_argument("--registry", help="registry with a 'reference' section (default: sequences_multi.yaml next to this file)")
    a = ap.parse_args()
    arms = load(a.results)
    if not arms:
        sys.exit("no <arm>/results.json under %s" % a.results)
    if a.kind == "fps":
        out = report_fps(arms)
    else:
        reference = None
        try:
            import yaml
            with open(a.registry or os.path.join(os.path.dirname(os.path.abspath(__file__)), "sequences_multi.yaml")) as fh:
                reference = yaml.safe_load(fh).get("reference")
        except Exception:  # PyYAML missing or no file: the thesis column is then empty
            pass
        out = report_multi(arms, reference)
    print("\n".join(out))


if __name__ == "__main__":
    main()
