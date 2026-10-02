#!/usr/bin/env python3
"""Author-comparable metrics from one SLAM run directory (slam.log plus dense outputs).

Zhang 2023 reports ATE, frame rate, the size of the dense cloud and of the Octomap, and
the per-keyframe running time of the dense stages (Tables IV-X). The trajectory error is
scored by run_benchmark.py / run_interleaved.py; everything else lives in the log the
SLAM binary prints and in the files it writes, which is what this module reads. Nothing
here touches a trajectory, so it cannot change a score.

    python3 evaluation/harness/log_metrics.py RUN_DIR [RUN_DIR ...]            # JSON
    python3 evaluation/harness/log_metrics.py --table RUN_DIR [RUN_DIR ...]    # one line each
    python3 evaluation/harness/log_metrics.py --self-test [--logs results_root]

The parser is tolerant on purpose: the binary's log format grows (new summary lines are
added by opt-in settings), so a line it does not know is ignored, a line it knows with
extra text still parses, and a metric it cannot find is None -- never 0. A count of 0 means
"the log is complete and the event did not occur", None means "this log cannot tell".

Keys (all flat, so they can be addressed as log_metrics.<key> by stats_compare.py):
  tracking_time_mean_s / tracking_time_median_s   printed by the example executables
        (rgbd_tum, stereo_kitti); stereo_euroc prints none. tracking_time_decimals is the
        number of decimals actually printed: a value of 2 or less is the stream-state
        rounding of the dense arms (std::fixed << setprecision(2) leaks into later
        output), flagged by tracking_time_low_precision.
  keyframes_total, maps_in_atlas       "Map N has K KFs" lines, summed.
  loops_detected, merges_detected      "*Loop detected" / "*Merge detected".
  local_mapping_stops                  "Local Mapping STOP" (loop/merge bookkeeping).
  map_inits, reinit_count              "New Map created" lines; re-inits = inits - 1.
  track_local_map_failures, relocalized, frames_set_lost   lost-tracking evidence at the
        default verbosity ("Fail to track local map!", "Relocalized!!", "N Frames set to lost").
  final_gba_maps, final_gba_done_maps, final_gba_seconds   C1 (GlobalBA.final) lines.
  shutdown_waited_s, shutdown_waited_lines                  "Shutdown: waited X s" (syncShutdown).
  shutdown_gave_up                     "[System.syncShutdown] WARNING: gave up waiting": the wait hit its
        bound, so the poses saved afterwards may still have been changing -- a run to look at.
  voxel_filter_calls, voxel_filter_overflows, voxel_filter_slabbed, voxel_filter_unfiltered,
  voxel_filter_seconds, voxel_safe, voxel_filter_unfiltered_calls, voxel_filter_summary,
  voxel_overflow_warnings
        the global-voxel-filter summary line, in either of its spellings ("Global voxel filter
        (voxelSafe=1): 132 calls, 120 would overflow pcl's int32 grid, 120 handled by slabs, 273.30 s"
        as the C++ prints it today; "Dense global voxel filter: 134 calls, 7 overflows ..."), and the raw
        pcl::VoxelGrid "Leaf size is too small ..." warnings counted independently of it.
        overflows = calls that pcl::VoxelGrid would refuse (it then returns the cloud UNFILTERED);
        unfiltered_calls = how many of them stayed unfiltered: all of them with voxelSafe=0, the
        "N left unfiltered" ones (0 if the line has none) with voxelSafe=1; None when it cannot be told.
  priority_boosts_refused              "Priority boosts refused: N" (the dense thread could not leave
        SCHED_IDLE); 0 when the dense timing block was printed without that line, None otherwise.
  dense_cloud_points, dense_octomap_nodes, dense_kf_dropped, dense_finalize_s,
  dense_timing_keyframes, dense_timing_ms (all stages), dense_{depth,voxel,map_update,
  octomap,total}_ms
        the "[Dense] saved ..." lines and the "Dense Reconstruction timing" block.
  files{cloud_pcd_bytes, octomap_ot_bytes, octomap_bt_bytes, pcd_points_header,
        compression_ratio_pcd_over_ot}   from the run directory; None once pruned.
  complete, crash, warnings            did the run reach Shutdown / save a trajectory, the
        first crash signature found, and what the parser could not make sense of.
"""

import argparse
import glob
import json
import os
import re
import sys

SCHEMA = 1

NUM = r"[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"

RE_TRACK = re.compile(r"^\s*(mean|median)\s+tracking\s+time\s*(?:[\[(]\s*(ms|s)\s*[\])])?\s*[:=]\s*(" + NUM + r")\s*(ms|s)?\s*$",
                      re.I)
RE_MAP_KFS = re.compile(r"^\s*Map\s+\d+\s+has\s+(\d+)\s+KFs")
RE_ATLAS_MAPS = re.compile(r"^There are\s+(\d+)\s+maps in the atlas")
RE_NEW_MAP = re.compile(r"^New Map created with\s+(\d+)\s+points")
RE_FRAMES_LOST = re.compile(r"^(\d+)\s+Frames set to lost")
RE_FINAL_GBA_ON = re.compile(r"^Final global bundle adjustment on map\s+\d+")
RE_FINAL_GBA_DONE = re.compile(r"^Final global bundle adjustment done\s*\((\d+)\s+map")
RE_FINAL_GBA_TIME = re.compile(r"final global bundle adjustment.*?(?:took|in|:)\s*(" + NUM + r")\s*(ms|s)\b", re.I)
RE_WAITED = re.compile(r"\bShutdown\b.*?\bwaited\b\s*[:=]?\s*(" + NUM + r")\s*(ms|s|sec|secs|seconds)?", re.I)
RE_VOXEL_SUMMARY = re.compile(r"\b(?:Dense\s+)?global\s+voxel\s+filter\b(.*)$", re.I)
RE_SYNC_GAVE_UP = re.compile(r"\[System\.syncShutdown\]\s*WARNING:\s*gave up waiting", re.I)
RE_BOOSTS_REFUSED = re.compile(r"^\s*Priority boosts refused\s*:\s*(\d+)")
RE_PCL_OVERFLOW = re.compile(r"Leaf size is too small for the input dataset\.?\s*Integer indices would overflow", re.I)
RE_SAVED_POINTS = re.compile(r"^\[Dense\]\s+saved\s+(\d+)\s+points\s+to\s+(\S+)")
RE_SAVED_OCTOMAP = re.compile(r"^\[Dense\]\s+saved octomap\s*\((\d+)\s+nodes\)\s+to\s+(\S+)")
RE_SAVED_BT = re.compile(r"^\[Dense\]\s+saved occupancy-only octomap to\s+(\S+)")
RE_TIMING_HEAD = re.compile(r"^Dense Reconstruction timing over\s+(\d+)\s+keyframes")
RE_TIMING_ROW = re.compile(r"^\s+([A-Za-z][A-Za-z0-9 /()._-]*?)\s*:\s*(" + NUM + r")\s*$")
RE_DROPPED = re.compile(r"Keyframes dropped\s*(?:\([^)]*\))?\s*:\s*(\d+)")
RE_FINALIZE_ROW = re.compile(r"Final-pose rebuild at Save\s*\(s\)\s*:\s*(" + NUM + r")")
RE_FINALIZE_LINE = re.compile(r"^\[Dense\]\s+final-pose reconstruction of\s+(\d+)\s+keyframes took\s+(" + NUM + r")\s*s")
RE_OFFLINE_KFS = re.compile(r"^\[Dense\]\s+keyframes:\s+(\d+)\s+stored,\s+(\d+)\s+culled.*?,\s+(\d+)\s+lost.*?,\s+(\d+)\s+gave an empty")
RE_EXTENSIONS = re.compile(r"^\s*extensions\s*:\s*(.*)$")
RE_COMPRESSION = re.compile(r"->\s*(" + NUM + r")x\s*\[([^\]]*)\]")

CRASH_SIGNATURES = (
    ("sophus", re.compile(r"Sophus ensure failed")),
    ("terminate", re.compile(r"terminate called")),
    ("segfault", re.compile(r"Segmentation fault|SIGSEGV")),
    ("abort", re.compile(r"\bAborted\b|SIGABRT")),
    ("bad_alloc", re.compile(r"std::bad_alloc|Cannot allocate memory")),
    ("double_free", re.compile(r"double free|corrupted (?:size|double-linked)")),
    ("harness_timeout", re.compile(r"\*\*\* TIMEOUT after")),
    ("harness_exec", re.compile(r"\*\*\* could not execute")),
)

TIMING_ALIASES = {
    "depth_acquisition": "dense_depth_ms",
    "voxel_filtering": "dense_voxel_ms",
    "map_update": "dense_map_update_ms",
    "octomap_conversion": "dense_octomap_ms",
    "total": "dense_total_ms",
}


def _slug(label):
    return re.sub(r"[^a-z0-9]+", "_", label.lower()).strip("_")


def _seconds(value, unit):
    """Value in the unit the log printed (default seconds) -> seconds."""
    v = float(value)
    return v / 1000.0 if unit and unit.lower() == "ms" else v


def _decimals(text):
    """Digits after the decimal point of the number as printed; None for exponent forms."""
    if "e" in text.lower():
        return None
    return len(text.split(".", 1)[1]) if "." in text else 0


def _kv_ints(text):
    """Integer facts of a free-form summary line: '12 calls', 'calls=12', 'calls: 12'."""
    out = {}
    for m in re.finditer(r"(\d+)\s+([A-Za-z][A-Za-z_]*)", text):
        out.setdefault(m.group(2).lower(), int(m.group(1)))
    for m in re.finditer(r"([A-Za-z][A-Za-z_]*)\s*[=:]\s*(\d+)(?![\d.])", text):
        out[m.group(1).lower()] = int(m.group(2))
    return out


def _pick(facts, stems):
    """First fact whose name starts with one of the stems (calls -> call, calls)."""
    for key, val in facts.items():
        if any(key.startswith(s) for s in stems):
            return val
    return None


def _first_int(rest, *patterns):
    for pat in patterns:
        hit = re.search(pat, rest, re.I)
        if hit:
            return int(hit.group(1))
    return None


def _voxel_facts(rest):
    """Facts of one global-voxel-filter summary line (the text after 'global voxel filter'), whatever the
    wording: '132 calls', 'calls=12', '120 would overflow', 'overflowed: 0', '120 handled by slabs',
    '3 left unfiltered', '273.30 s'. A fact the line does not state is None; the generic 'N word' /
    'word=N' scan fills in calls and overflows for spellings not listed here. A number that is the
    value of a key ('calls=12 overflows=0': the 12) is never read as the count of the next word."""
    n = r"(?<![\w=:.])(\d+)"
    f = {
        "calls": _first_int(rest, n + r"\s+(?:calls?|invocations?|passes)\b", r"\b(?:calls?|invocations?|passes)\s*[=:]\s*(\d+)"),
        "overflows": _first_int(rest, n + r"\s+(?:would\s+)?overflow(?:s|ed)?\b", r"\boverflow(?:s|ed)?\s*[=:]\s*(\d+)",
                                n + r"\s+(?:fallbacks?|unsafe|failed)\b"),
        "slabbed": _first_int(rest, n + r"\s+(?:handled\s+by\s+)?slab(?:s|bed)?\b", r"\bslab(?:s|bed)?\s*[=:]\s*(\d+)"),
        "unfiltered": _first_int(rest, n + r"\s+(?:left\s+)?unfiltered\b", r"\bunfiltered\s*[=:]\s*(\d+)"),
        "safe": _first_int(rest, r"voxelSafe\s*[=:]\s*(\d)"),
        "seconds": None,
    }
    sec = re.search(r"(" + NUM + r")\s*s(?:ec(?:onds?)?)?\b(?:\s+total)?\s*$", rest.strip(), re.I)
    if sec:
        f["seconds"] = float(sec.group(1))
    if f["calls"] is None and f["overflows"] is None:
        generic = _kv_ints(rest)
        f["calls"] = _pick(generic, ("call", "invocation", "pass"))
        f["overflows"] = _pick(generic, ("overflow", "fallback", "unsafe", "failed"))
    return f


def parse_log(text):
    """Metrics from the text of one slam.log. Unknown lines are ignored."""
    m = {
        "tracking_time_mean_s": None, "tracking_time_median_s": None,
        "tracking_time_decimals": None, "tracking_time_low_precision": None,
        "keyframes_total": None, "maps_in_atlas": None,
        "loops_detected": 0, "merges_detected": 0, "local_mapping_stops": 0,
        "map_inits": 0, "reinit_count": None, "init_points_first": None,
        "track_local_map_failures": 0, "relocalized": 0, "frames_set_lost": 0,
        "final_gba_maps": 0, "final_gba_done_maps": None, "final_gba_seconds": None,
        "shutdown_waited_s": None, "shutdown_waited_lines": 0, "shutdown_gave_up": 0,
        "voxel_filter_calls": None, "voxel_filter_overflows": None,
        "voxel_filter_slabbed": None, "voxel_filter_unfiltered": None,
        "voxel_filter_seconds": None, "voxel_safe": None, "voxel_filter_unfiltered_calls": None,
        "voxel_filter_summary": None, "voxel_filter_facts": None,
        "voxel_overflow_warnings": 0, "priority_boosts_refused": None,
        "dense_enabled": False, "dense_extensions": None,
        "dense_cloud_points": None, "dense_octomap_nodes": None,
        "dense_kf_dropped": None, "dense_finalize_s": None,
        "dense_timing_keyframes": None, "dense_timing_ms": None,
        "dense_offline_stored": None, "dense_offline_culled": None,
        "dense_offline_lost": None, "dense_offline_empty": None,
        "dense_compression": None,
        "reached_shutdown": False, "saved_trajectory": False, "crash": None,
    }
    for key in TIMING_ALIASES.values():
        m[key] = None
    warnings = []
    kfs, atlas_maps, decimals = [], [], []
    voxel_lines, voxel_facts = [], {}
    waited = []
    final_gba_times = []
    in_timing = False
    timing = None

    for raw in text.splitlines():
        line = raw.rstrip("\r")
        s = line.strip()

        if in_timing:
            row = RE_TIMING_ROW.match(line)
            if row and not RE_DROPPED.search(line) and not RE_FINALIZE_ROW.search(line):
                timing[_slug(row.group(1))] = float(row.group(2))
                continue
            in_timing = False  # first line that is not a stage row ends the block

        if not s:
            continue

        t = RE_TRACK.match(line)
        if t:
            kind, unit_a, value, unit_b = t.groups()
            unit = unit_b or unit_a
            m["tracking_time_%s_s" % kind.lower()] = _seconds(value, unit)
            if not unit:
                decimals.append(_decimals(value))
            continue

        if s.startswith("*Loop detected"):
            m["loops_detected"] += 1
        elif s.startswith("*Merge detected"):
            m["merges_detected"] += 1
        elif s == "Local Mapping STOP":
            m["local_mapping_stops"] += 1
        elif s.startswith("Fail to track local map"):
            m["track_local_map_failures"] += 1
        elif s.startswith("Relocalized!!"):
            m["relocalized"] += 1
        elif s == "Shutdown" or s.startswith("Shutdown:"):
            m["reached_shutdown"] = True
        elif s.startswith("Saving") and "trajectory" in s:
            m["saved_trajectory"] = True

        nm = RE_NEW_MAP.match(s)
        if nm:
            m["map_inits"] += 1
            if m["init_points_first"] is None:
                m["init_points_first"] = int(nm.group(1))
        fl = RE_FRAMES_LOST.match(s)
        if fl:
            m["frames_set_lost"] += int(fl.group(1))
        mk = RE_MAP_KFS.match(line)
        if mk:
            kfs.append(int(mk.group(1)))
        am = RE_ATLAS_MAPS.match(s)
        if am:
            atlas_maps.append(int(am.group(1)))

        if RE_PCL_OVERFLOW.search(line):
            m["voxel_overflow_warnings"] += 1

        if RE_FINAL_GBA_ON.match(s):
            m["final_gba_maps"] += 1
        gd = RE_FINAL_GBA_DONE.match(s)
        if gd:
            m["final_gba_done_maps"] = (m["final_gba_done_maps"] or 0) + int(gd.group(1))
        if "final global bundle adjustment" in s.lower():
            gt = RE_FINAL_GBA_TIME.search(s)
            if gt:
                final_gba_times.append(_seconds(gt.group(1), gt.group(2)))

        if "waited" in s and "Shutdown" in s:
            w = RE_WAITED.search(s)
            if w:
                waited.append(_seconds(w.group(1), w.group(2)))
            else:
                warnings.append("unparsed 'Shutdown ... waited' line: %s" % s[:120])
            m["shutdown_waited_lines"] += 1

        vm = RE_VOXEL_SUMMARY.search(s)
        if vm:
            voxel_lines.append(s)
            for k, v in _voxel_facts(vm.group(1)).items():
                if v is not None:  # several summaries (not expected) add up; the on/off flag does not
                    voxel_facts[k] = max(voxel_facts.get(k, 0), v) if k == "safe" else voxel_facts.get(k, 0) + v
        if RE_SYNC_GAVE_UP.search(line):
            m["shutdown_gave_up"] += 1
        br = RE_BOOSTS_REFUSED.match(line)
        if br:
            m["priority_boosts_refused"] = int(br.group(1))

        if s.startswith("[Dense] reconstruction enabled"):
            m["dense_enabled"] = True
        ext = RE_EXTENSIONS.match(line)
        if ext:
            m["dense_extensions"] = dict(kv.split("=", 1) for kv in ext.group(1).split() if "=" in kv)
        sp = RE_SAVED_POINTS.match(s)
        if sp:
            m["dense_cloud_points"] = int(sp.group(1))
        so = RE_SAVED_OCTOMAP.match(s)
        if so:
            m["dense_octomap_nodes"] = int(so.group(1))
        th = RE_TIMING_HEAD.match(s)
        if th:
            m["dense_timing_keyframes"] = int(th.group(1))
            timing = {}
            in_timing = True
        dr = RE_DROPPED.search(line)
        if dr:
            m["dense_kf_dropped"] = int(dr.group(1))
        fr = RE_FINALIZE_ROW.search(line)
        if fr:
            m["dense_finalize_s"] = float(fr.group(1))
        fl2 = RE_FINALIZE_LINE.match(s)
        if fl2 and m["dense_finalize_s"] is None:
            m["dense_finalize_s"] = float(fl2.group(2))
        ok = RE_OFFLINE_KFS.match(s)
        if ok:
            (m["dense_offline_stored"], m["dense_offline_culled"],
             m["dense_offline_lost"], m["dense_offline_empty"]) = (int(g) for g in ok.groups())
        cm = RE_COMPRESSION.search(line)
        if cm:
            m["dense_compression"] = m["dense_compression"] or {}
            m["dense_compression"][_slug(cm.group(2))] = float(cm.group(1))

        if m["crash"] is None:
            for name, rx in CRASH_SIGNATURES:
                if rx.search(line):
                    m["crash"] = name
                    break

        if timing is not None and not in_timing and m["dense_timing_ms"] is None:
            m["dense_timing_ms"] = timing

    if timing is not None and m["dense_timing_ms"] is None:
        m["dense_timing_ms"] = timing
    if m["dense_timing_ms"]:
        for label, key in TIMING_ALIASES.items():
            if label in m["dense_timing_ms"]:
                m[key] = m["dense_timing_ms"][label]

    if kfs:
        m["keyframes_total"] = sum(kfs)
    if atlas_maps:
        m["maps_in_atlas"] = atlas_maps[-1]
    if m["map_inits"]:
        m["reinit_count"] = m["map_inits"] - 1
    if decimals and all(d is not None for d in decimals):
        m["tracking_time_decimals"] = min(decimals)
        m["tracking_time_low_precision"] = min(decimals) <= 2
    if final_gba_times:
        m["final_gba_seconds"] = sum(final_gba_times)
    if waited:
        m["shutdown_waited_s"] = sum(waited)
    if voxel_lines:
        m["voxel_filter_summary"] = " | ".join(voxel_lines)
        m["voxel_filter_facts"] = voxel_facts
        m["voxel_filter_calls"] = voxel_facts.get("calls")
        m["voxel_filter_overflows"] = voxel_facts.get("overflows")
        m["voxel_filter_slabbed"] = voxel_facts.get("slabbed")
        m["voxel_filter_unfiltered"] = voxel_facts.get("unfiltered")
        m["voxel_filter_seconds"] = voxel_facts.get("seconds")
        m["voxel_safe"] = voxel_facts.get("safe")
        if m["voxel_filter_calls"] is None and m["voxel_filter_overflows"] is None:
            warnings.append("global voxel filter line without calls/overflow counts: %s" % voxel_lines[0][:160])
    if m["voxel_safe"] is None and m["dense_extensions"] and m["dense_extensions"].get("voxelSafe") in ("0", "1"):
        m["voxel_safe"] = int(m["dense_extensions"]["voxelSafe"])
    if m["voxel_filter_overflows"] is not None:
        if m["voxel_safe"] == 0:
            m["voxel_filter_unfiltered_calls"] = m["voxel_filter_overflows"]  # pcl refused them and returned the input
        elif m["voxel_safe"] == 1:
            m["voxel_filter_unfiltered_calls"] = m["voxel_filter_unfiltered"] or 0  # the line names them only when > 0
    if m["priority_boosts_refused"] is None and m["dense_timing_ms"] is not None:
        m["priority_boosts_refused"] = 0
    return m, warnings


def _pcd_header(path):
    """(points, data_kind) from a PCD header; (None, None) if it cannot be read."""
    try:
        with open(path, "rb") as fh:
            head = fh.read(4096).decode("ascii", "replace")
    except OSError:
        return None, None
    pts = re.search(r"^POINTS\s+(\d+)", head, re.M)
    kind = re.search(r"^DATA\s+(\w+)", head, re.M)
    return (int(pts.group(1)) if pts else None), (kind.group(1) if kind else None)


def file_metrics(run_dir):
    """Sizes of the dense outputs in a run directory (None when absent, e.g. pruned)."""
    out = {"cloud_pcd_bytes": None, "octomap_ot_bytes": None, "octomap_bt_bytes": None,
           "pcd_points_header": None, "pcd_data": None, "compression_ratio_pcd_over_ot": None}
    if not run_dir or not os.path.isdir(run_dir):
        return out
    pcds = sorted(p for p in glob.glob(os.path.join(run_dir, "*.pcd")) if ".tmp." not in os.path.basename(p))
    cloud = [p for p in pcds if p.endswith("_cloud.pcd")] or pcds
    if cloud:
        out["cloud_pcd_bytes"] = os.path.getsize(cloud[0])
        out["pcd_points_header"], out["pcd_data"] = _pcd_header(cloud[0])
    ots = sorted(glob.glob(os.path.join(run_dir, "*.ot")))
    if ots:
        out["octomap_ot_bytes"] = os.path.getsize(ots[0])
    bts = sorted(glob.glob(os.path.join(run_dir, "*.bt")))
    if bts:
        out["octomap_bt_bytes"] = os.path.getsize(bts[0])
    if out["cloud_pcd_bytes"] and out["octomap_ot_bytes"]:
        out["compression_ratio_pcd_over_ot"] = out["cloud_pcd_bytes"] / float(out["octomap_ot_bytes"])
    return out


def _unknown():
    """Every key of parse_log with the counters set to None: the log cannot tell."""
    blank = parse_log("")[0]
    return {k: (None if (isinstance(v, int) and not isinstance(v, bool)) else v) for k, v in blank.items()}


def parse_run_dir(run_dir, log_name="slam.log"):
    """All metrics of one run directory. Never raises for a missing or odd log."""
    log_path = os.path.join(run_dir, log_name)
    res = {"schema": SCHEMA, "log": log_name}
    try:
        with open(log_path, "rb") as fh:
            raw = fh.read()
    except OSError as exc:
        res.update(_unknown())
        res.update(complete=False, warnings=["cannot read %s: %s" % (log_path, exc)])
        res["files"] = file_metrics(run_dir)
        return res
    text = raw.decode("utf-8", "replace")
    if not text.strip():
        res.update(_unknown())
        res.update(complete=False, log_bytes=len(raw), log_lines=0, warnings=["empty log"])
        res["files"] = file_metrics(run_dir)
        return res
    metrics, warnings = parse_log(text)
    res.update(metrics)
    res["log_bytes"] = len(raw)
    res["log_lines"] = text.count("\n")
    # "complete": the binary ran to the end of main(), not merely past Shutdown().
    res["complete"] = bool(metrics["reached_shutdown"] and metrics["saved_trajectory"])
    if not metrics["reached_shutdown"]:
        warnings.append("no 'Shutdown' line: the run did not finish (crash, kill or timeout)")
    if metrics["dense_enabled"] and metrics["dense_timing_ms"] is None and metrics["reached_shutdown"]:
        warnings.append("dense enabled but no 'Dense Reconstruction timing' block")
    if metrics["tracking_time_low_precision"]:
        warnings.append("tracking time printed with %d decimals (stream-state rounding)"
                        % metrics["tracking_time_decimals"])
    res["warnings"] = warnings
    res["files"] = file_metrics(run_dir)
    f = res["files"]
    if f["pcd_points_header"] is not None and metrics["dense_cloud_points"] is not None \
            and f["pcd_points_header"] != metrics["dense_cloud_points"]:
        res["warnings"].append("PCD header says %d points, log says %d"
                               % (f["pcd_points_header"], metrics["dense_cloud_points"]))
    return res


# ---------------------------------------------------------------------------------------
# self-test
# ---------------------------------------------------------------------------------------

SAMPLE_IMP = """\
[Dense] reconstruction enabled
        voxel resolution   : 0.019999999552965164 m
        extensions         : queueLimit=3 lowPriority=1 mode=online reproject=0 octomapAtEnd=0 outlierRemoval=0 wls=0
First KF:0; Map init KF:0
New Map created with 529 points
*Loop detected
Local Mapping STOP
Local Mapping RELEASE
Local Mapping STOP
Local Mapping RELEASE
Fail to track local map!
Fail to track local map!
Relocalized!!
[pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.
[pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow.
Shutdown
Final global bundle adjustment on map 0 ...
Final global bundle adjustment done (1 map(s))
[Dense] saved 9348506 points to ./euroc_cloud.pcd
[Dense] saved octomap (2926257 nodes) to ./euroc_octomap.ot

Dense Reconstruction timing over 153 keyframes (mean, ms)
  Depth Acquisition : 188.98
  Voxel Filtering   : 14.72
  Map Update        : 10.41
  Octomap Conversion: 19.42
  Total             : 233.56
  Keyframes dropped (queueLimit=3): 2

Saving trajectory to f_run00.txt ...
There are 1 maps in the atlas
  Map 0 has 150 KFs

End of saving trajectory to f_run00.txt ...
"""


def run_self_test(logs_root=None):
    failures = []

    def check(name, cond, info=""):
        print(("PASS " if cond else "FAIL ") + name + (("  " + info) if info else ""))
        if not cond:
            failures.append(name)

    m, w = parse_log(SAMPLE_IMP)
    check("dense imp sample: loop / stops / failures / relocalized",
          (m["loops_detected"], m["local_mapping_stops"], m["track_local_map_failures"], m["relocalized"]) == (1, 2, 2, 1))
    check("dense imp sample: raw PCL overflow warnings counted", m["voxel_overflow_warnings"] == 2)
    check("dense imp sample: cloud points / octomap nodes",
          (m["dense_cloud_points"], m["dense_octomap_nodes"]) == (9348506, 2926257))
    check("dense imp sample: timing block, five stages and the alias keys",
          m["dense_timing_keyframes"] == 153 and m["dense_total_ms"] == 233.56 and m["dense_depth_ms"] == 188.98
          and m["dense_voxel_ms"] == 14.72 and m["dense_map_update_ms"] == 10.41 and m["dense_octomap_ms"] == 19.42
          and len(m["dense_timing_ms"]) == 5)
    check("dense imp sample: dropped keyframes", m["dense_kf_dropped"] == 2)
    check("dense imp sample: final GBA lines", (m["final_gba_maps"], m["final_gba_done_maps"]) == (1, 1))
    check("dense imp sample: extensions line", m["dense_extensions"]["queueLimit"] == "3" and m["dense_extensions"]["mode"] == "online")
    check("dense imp sample: keyframes / init / re-init", (m["keyframes_total"], m["map_inits"], m["reinit_count"]) == (150, 1, 0))
    check("a count is 0, an absent fact is None: no waited line, no summary, no tracking time",
          m["shutdown_waited_s"] is None and m["voxel_filter_calls"] is None and m["tracking_time_mean_s"] is None)

    m, _ = parse_log("median tracking time: 0.0388889\nmean tracking time: 0.0370427\n")
    check("tracking time, default stream precision", m["tracking_time_median_s"] == 0.0388889
          and m["tracking_time_decimals"] == 7 and m["tracking_time_low_precision"] is False)
    m, _ = parse_log("median tracking time: 0.03\nmean tracking time: 0.03\n")
    check("tracking time, fixed 2-decimal leak is flagged", m["tracking_time_decimals"] == 2 and m["tracking_time_low_precision"])
    m, _ = parse_log("mean tracking time: 0.037042701244354248\nmedian tracking time: 0.034000000000000002\n")
    check("tracking time, full precision", m["tracking_time_decimals"] == 18 and not m["tracking_time_low_precision"]
          and m["tracking_time_mean_s"] == 0.037042701244354248)
    m, _ = parse_log("mean tracking time [ms]: 37.04\n")
    check("tracking time with an explicit ms unit", abs(m["tracking_time_mean_s"] - 0.03704) < 1e-12)

    for text, expect in (("Shutdown: waited 3.25 s for LocalMapping, LoopClosing and GBA", 3.25),
                         ("Shutdown: waited 120 ms", 0.12),
                         ("Shutdown: waited 2 seconds", 2.0),
                         ("Shutdown: waited 1.5s\nShutdown: waited 0.5 s", 2.0)):
        m, w = parse_log(text)
        check("Shutdown waited: %r" % text.splitlines()[0][:40], m["shutdown_waited_s"] == expect and not w,
              "got %r" % m["shutdown_waited_s"])
    m, w = parse_log("Shutdown: waited for the backend\n")
    check("Shutdown waited without a number: counted, warned, not guessed",
          m["shutdown_waited_s"] is None and m["shutdown_waited_lines"] == 1 and len(w) == 1)

    for text, calls, over in (("Dense global voxel filter: 134 calls, 7 overflows, 0.9 s total", 134, 7),
                              ("Dense global voxel filter: calls=12 overflows=0", 12, 0),
                              ("Dense global voxel filter: 12 calls (3 overflow, 9 ok)", 12, 3),
                              ("Dense global voxel filter: calls: 50, overflowed: 0, safe path used 0 times", 50, 0),
                              ("[Dense] Dense global voxel filter: 40 invocations, 0 overflows", 40, 0)):
        m, w = parse_log(text)
        check("voxel summary: %r" % text[:58], (m["voxel_filter_calls"], m["voxel_filter_overflows"]) == (calls, over),
              "got %r" % ((m["voxel_filter_calls"], m["voxel_filter_overflows"]),))
    m, w = parse_log("Dense global voxel filter: done\n")
    check("voxel summary without numbers: kept verbatim, warned", m["voxel_filter_summary"] is not None
          and m["voxel_filter_calls"] is None and len(w) == 1)

    # the lines as the C++ prints them (PointCloudMapping::PrintTimingSummary, System::Shutdown)
    for text, want in (
            ("  Global voxel filter (voxelSafe=1): 132 calls, 120 would overflow pcl's int32 grid, 120 handled by slabs, 273.30 s",
             dict(voxel_filter_calls=132, voxel_filter_overflows=120, voxel_filter_slabbed=120, voxel_filter_unfiltered=None,
                  voxel_safe=1, voxel_filter_seconds=273.3, voxel_filter_unfiltered_calls=0)),
            ("  Global voxel filter (voxelSafe=1): 5 calls, 4 would overflow pcl's int32 grid, 3 handled by slabs, 1 left unfiltered, 12.50 s",
             dict(voxel_filter_calls=5, voxel_filter_overflows=4, voxel_filter_slabbed=3, voxel_filter_unfiltered=1,
                  voxel_safe=1, voxel_filter_seconds=12.5, voxel_filter_unfiltered_calls=1)),
            ("  Global voxel filter (voxelSafe=0): 132 calls, 120 would overflow pcl's int32 grid, 0 handled by slabs, 80.10 s",
             dict(voxel_filter_calls=132, voxel_filter_overflows=120, voxel_filter_slabbed=0, voxel_safe=0,
                  voxel_filter_seconds=80.1, voxel_filter_unfiltered_calls=120)),
            ("  Global voxel filter (voxelSafe=0): 40 calls, 0 would overflow pcl's int32 grid, 0 handled by slabs, 1.00 s",
             dict(voxel_filter_calls=40, voxel_filter_overflows=0, voxel_safe=0, voxel_filter_unfiltered_calls=0))):
        m, w = parse_log(text)
        bad = {k: (m[k], v) for k, v in want.items() if m[k] != v}
        check("voxel summary as printed by the C++: %s" % text.strip()[19:50], not bad and not w, "got/want %r" % bad)
    m, _ = parse_log("[Dense] reconstruction enabled\n        extensions         : queueLimit=3 lowPriority=1 mode=online reproject=0 "
                     "octomapAtEnd=0 outlierRemoval=0 wls=0 voxelSafe=1\n")
    check("voxelSafe from the extensions line when there is no summary", m["voxel_safe"] == 1 and m["voxel_filter_calls"] is None)

    m, _ = parse_log("[System.syncShutdown] WARNING: gave up waiting after 1800 s (LocalMapping finished=1, LoopClosing finished=1, "
                     "GBA running=1); continuing, the poses saved from here on may still be changing.\n"
                     "Shutdown: waited 1800.00 s for LocalMapping/LoopClosing/GBA\n")
    check("sync-shutdown give-up warning is counted and the wait is still read",
          m["shutdown_gave_up"] == 1 and m["shutdown_waited_s"] == 1800.0 and m["shutdown_waited_lines"] == 1)
    m, _ = parse_log("Shutdown\nShutdown: waited 0.01 s for LocalMapping/LoopClosing/GBA\n")
    check("no give-up warning: 0, not None", m["shutdown_gave_up"] == 0 and m["shutdown_waited_s"] == 0.01)
    block = "Dense Reconstruction timing over 3 keyframes (mean, ms)\n  Depth Acquisition : 1.00\n  Total             : 2.00\n"
    m, _ = parse_log(block + "  Priority boosts refused: 4 (dense thread stayed SCHED_IDLE while holding mMutexQueue/mMutexCloud)\n")
    m2, _ = parse_log(block)
    m3, _ = parse_log("Shutdown\n")
    check("priority boosts refused: the number; 0 when the timing block has no such line; None without a block",
          (m["priority_boosts_refused"], m2["priority_boosts_refused"], m3["priority_boosts_refused"]) == (4, 0, None)
          and m["dense_total_ms"] == 2.0)

    m, _ = parse_log("Final global bundle adjustment on map 0 ...\nFinal global bundle adjustment done (1 map(s)) in 12.5 s\n")
    check("final GBA duration, if the binary prints one", m["final_gba_seconds"] == 12.5)

    m, _ = parse_log("New Map created with 937 points\nNew Map created with 400 points\nCreation of new map with id: 1\n12 Frames set to lost\n")
    check("re-init and lost frames", (m["map_inits"], m["reinit_count"], m["frames_set_lost"], m["init_points_first"]) == (2, 1, 12, 937))

    m, _ = parse_log("[Dense] offline: building clouds for 156 keyframes ...\n"
                     "[Dense] keyframes: 156 stored, 6 culled (rescued via parent), 0 lost (no live ancestor), 2 gave an empty cloud\n"
                     "[Dense] final-pose reconstruction of 156 keyframes took 37.83 s\n"
                     "\nDense Reconstruction timing over 156 keyframes (mean, ms)\n  Depth Acquisition : 164.42\n"
                     "  Global Voxel Filtering: 3.50\n  Total             : 178.24\n  Final-pose rebuild at Save (s): 37.83\n")
    check("offline dense lines and an unknown extra stage row",
          (m["dense_offline_stored"], m["dense_offline_culled"], m["dense_offline_lost"], m["dense_offline_empty"]) == (156, 6, 0, 2)
          and m["dense_finalize_s"] == 37.83 and m["dense_timing_ms"]["global_voxel_filtering"] == 3.5
          and "final_pose_rebuild_at_save_s" not in m["dense_timing_ms"])

    m, _ = parse_log("[Dense] compression report (sizes in MiB, ratio = cloud / octomap)\n"
                     "  binary XYZRGB .pcd 10.00  vs colour .ot   4.00  -> 2.50x   [same format: binary, colour]\n")
    check("compression report line", m["dense_compression"] == {"same_format_binary_colour": 2.5})

    m, _ = parse_log("Sophus ensure failed in function 'x', file 'so3.hpp', line 614.\n")
    check("crash signature", m["crash"] == "sophus")
    r = parse_run_dir("/nonexistent/run")
    check("missing run directory does not raise", r["complete"] is False and r["warnings"])

    import tempfile
    with tempfile.TemporaryDirectory() as d:
        with open(os.path.join(d, "slam.log"), "w") as fh:
            fh.write("")
        r = parse_run_dir(d)
        check("empty log is reported, not parsed as zeros", r["complete"] is False and "empty log" in r["warnings"])
        with open(os.path.join(d, "slam.log"), "w") as fh:
            fh.write(SAMPLE_IMP)
        with open(os.path.join(d, "euroc_cloud.pcd"), "wb") as fh:
            fh.write(b"# .PCD v0.7\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n"
                     b"WIDTH 9348506\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 9348506\nDATA binary\n" + b"\0" * 64)
        with open(os.path.join(d, "euroc_octomap.ot"), "wb") as fh:
            fh.write(b"\0" * 32)
        r = parse_run_dir(d)
        f = r["files"]
        check("run dir with outputs: sizes and PCD header", f["cloud_pcd_bytes"] > 64 and f["octomap_ot_bytes"] == 32
              and f["pcd_points_header"] == 9348506 and f["pcd_data"] == "binary" and f["compression_ratio_pcd_over_ot"] > 1)
        check("header agrees with the log, so no warning about it", not any("PCD header" in x for x in r["warnings"]))
        check("complete run", r["complete"] is True)
        json.dumps(r)  # must be serialisable as is
        check("result is JSON-serialisable", True)

    if logs_root:
        run_real_log_checks(logs_root, check)
    print("\nself-test: %s" % ("ALL PASSED" if not failures else "FAILED: " + ", ".join(failures)))
    return 0 if not failures else 1


def run_real_log_checks(root, check):
    """Compare the parser with independent substring counts on every slam.log under root."""
    paths = sorted(glob.glob(os.path.join(root, "*", "*", "run*", "slam.log")))
    check("real logs found under %s" % root, len(paths) > 0, "%d logs" % len(paths))
    tot = {"over": 0, "loops": 0, "stops": 0, "fail": 0, "gba_done": 0, "timing": 0, "saved": 0, "new_map": 0}
    exp = {k: 0 for k in tot}
    mism = []
    n_complete = n_dense_nodrop = n_unknown = 0
    low_prec = 0
    for p in paths:
        r = parse_run_dir(os.path.dirname(p))
        with open(p, encoding="utf-8", errors="replace") as fh:
            text = fh.read()
        e = {"over": text.count("Integer indices would overflow"),
             "loops": sum(1 for ln in text.splitlines() if ln.strip().startswith("*Loop detected")),
             "stops": text.count("Local Mapping STOP"),
             "fail": text.count("Fail to track local map!"),
             "gba_done": text.count("Final global bundle adjustment done"),
             "timing": text.count("Dense Reconstruction timing over"),
             "saved": len(re.findall(r"\[Dense\] saved \d+ points", text)),
             "new_map": text.count("New Map created with")}
        got = {"over": r["voxel_overflow_warnings"], "loops": r["loops_detected"], "stops": r["local_mapping_stops"],
               "fail": r["track_local_map_failures"], "gba_done": 1 if r["final_gba_done_maps"] else 0,
               "timing": 1 if r["dense_timing_ms"] else 0, "saved": 1 if r["dense_cloud_points"] is not None else 0,
               "new_map": r["map_inits"]}
        if r["loops_detected"] is None:  # empty or unreadable log: the parser says "cannot tell", the text has no events
            n_unknown += 1
            got = {k: (0 if v is None else v) for k, v in got.items()}
        for k in tot:
            tot[k] += got[k]
            exp[k] += e[k]
            if got[k] != e[k]:
                mism.append((p, k, got[k], e[k]))
        n_complete += bool(r["complete"])
        low_prec += bool(r["tracking_time_low_precision"])
        n_dense_nodrop += r["dense_kf_dropped"] is None and r["dense_enabled"]
    check("real logs: parser counts equal independent substring counts (%d logs)" % len(paths), not mism,
          "; ".join("%s %s parsed %s vs %s" % m for m in mism[:3]))
    print("      totals parsed: " + ", ".join("%s=%d" % kv for kv in sorted(tot.items())))
    print("      complete runs %d of %d; empty/unreadable logs %d; low-precision tracking-time logs %d; "
          "dense logs without a drop line %d" % (n_complete, len(paths), n_unknown, low_prec, n_dense_nodrop))


# ---------------------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------------------

def _table_row(path, r):
    def f(v, spec="%.4g"):
        return "-" if v is None else spec % v
    return "%-48s %s trk=%s loops=%d reinit=%s waited=%s ovf_warn=%d pts=%s dens_ms=%s drop=%s" % (
        path[-48:], "ok  " if r.get("complete") else "INC ", f(r.get("tracking_time_mean_s")),
        r.get("loops_detected") or 0, r.get("reinit_count"), f(r.get("shutdown_waited_s")),
        r.get("voxel_overflow_warnings") or 0, r.get("dense_cloud_points"), f(r.get("dense_total_ms")),
        r.get("dense_kf_dropped"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dirs", nargs="*", help="run directories (each holds slam.log)")
    ap.add_argument("--table", action="store_true", help="one summary line per run instead of JSON")
    ap.add_argument("--log-name", default="slam.log")
    ap.add_argument("--self-test", action="store_true", help="run the built-in checks and exit")
    ap.add_argument("--logs", metavar="RESULTS_ROOT",
                    help="with --self-test: also check the parser against every <root>/*/*/run*/slam.log")
    a = ap.parse_args()
    if a.self_test:
        sys.exit(run_self_test(a.logs))
    if not a.run_dirs:
        ap.error("give at least one run directory (or --self-test)")
    out = {}
    for d in a.run_dirs:
        out[d] = parse_run_dir(d, a.log_name)
    if a.table:
        for d, r in out.items():
            print(_table_row(d, r))
    else:
        print(json.dumps(out if len(out) > 1 else next(iter(out.values())), indent=1))


if __name__ == "__main__":
    main()
