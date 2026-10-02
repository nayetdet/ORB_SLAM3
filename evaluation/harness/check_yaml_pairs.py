#!/usr/bin/env python3
"""Prove, from the settings files, that the arms of a comparison differ only where intended.

An arm is a config of sequences.yaml; its settings are the yaml file(s) it runs with. Comments
and layout are ignored: each file is read as a flat map key -> value (the OpenCV settings are
flat; the multi-line Stereo.T_c1_c2 matrix is joined into one value), numbers are compared as
numbers. For every pair the script prints exactly which keys differ, and fails if a key outside
the intended set differs, or if an intended key does not hold its intended value (for example
a "thesis-faithful" arm with an opt-in switch turned on).

    python3 evaluation/harness/check_yaml_pairs.py                 # the pilot pairs below
    python3 evaluation/harness/check_yaml_pairs.py --pair euroc_p_imp:euroc_p_c1:Dense.*
    python3 evaluation/harness/check_yaml_pairs.py --self-test

Needs only PyYAML (no numpy): it runs on the host. Exit status 0 = every pair is as intended.
"""

import argparse
import os
import re
import sys

try:
    import yaml
except ImportError:
    sys.exit("PyYAML is required: pip install pyyaml  (or apt install python3-yaml)")

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))

ABSENT = object()  # the key must not be in the file ...
OFF = object()     # ... or be absent / 0 (an opt-in switch left at its default)

# (arm, reference, intended differences). A rule entry ending in ".*" is a key prefix.
PAIRS = [
    ("euroc_p_base_b", "euroc_p_base", []),                          # A/A replicate: identical settings
    ("euroc_p_c1", "euroc_p_base", ["GlobalBA.final"]),              # baseline + final global BA
    ("euroc_p_faithful", "euroc_p_base", ["Dense.*"]),               # thesis-faithful dense thread
    ("euroc_p_imp", "euroc_p_base", ["Dense.*", "GlobalBA.final"]),  # dense_imp = Dense.* + final BA
    ("kitti_s_sync", "kitti_s_nosync", ["System.syncShutdown"]),     # shutdown-synchronisation check
]
# The same four-arm design on TUM RGB-D and KITTI (evaluation/PRE_REGISTRATION_tum_kitti.md).
for _p in ("tum_p", "kitti_p"):
    PAIRS += [(_p + "_c1", _p + "_base", ["GlobalBA.final"]),
              (_p + "_faithful", _p + "_base", ["Dense.*"]),
              (_p + "_imp", _p + "_base", ["Dense.*", "GlobalBA.final"])]

# Values that must hold whatever the pair says (key -> value) for every file of the arm.
COMMON_EUROC = {"ORBextractor.nFeatures": 1000, "System.UseViewer": 0, "System.syncShutdown": 1}
COMMON_TUM = dict(COMMON_EUROC)                                   # TUM{1,2,3}.yaml keep 1000 features
COMMON_KITTI = dict(COMMON_EUROC, **{"ORBextractor.nFeatures": 2000})  # KITTI*.yaml keep 2000 features
THESIS_FAITHFUL_DENSE = {
    "Dense.enabled": 1, "Dense.queueLimit": 0, "Dense.lowPriority": 0, "Dense.mode": "online",
    "Dense.reprojectOptimized": 0, "Dense.octomapAtEnd": 0, "Dense.octomapRayCast": 0, "Dense.outlierRemoval": 0,
    "Dense.wlsFilter": 0, "Dense.probabilisticMerge": 0, "Dense.voxelSafe": 0, "GlobalBA.final": OFF,
}


def arm_expectations(prefix, common):
    return {
        prefix + "_base": dict(common, **{"Dense.enabled": 0, "GlobalBA.final": OFF}),
        prefix + "_c1": dict(common, **{"Dense.enabled": 0, "GlobalBA.final": 1}),
        prefix + "_faithful": dict(common, **THESIS_FAITHFUL_DENSE),
        prefix + "_imp": dict(common, **{"Dense.enabled": 1, "Dense.queueLimit": 3, "Dense.lowPriority": 1,
                                         "Dense.mode": "online", "Dense.voxelSafe": 1, "GlobalBA.final": 1,
                                         "Dense.reprojectOptimized": 0, "Dense.octomapAtEnd": 0}),
    }


EXPECT = {
    "euroc_p_base_b": dict(COMMON_EUROC, **{"Dense.enabled": 0, "GlobalBA.final": OFF}),
    "kitti_s_nosync": {"System.syncShutdown": OFF, "Dense.enabled": 0, "GlobalBA.final": OFF},
    "kitti_s_sync": {"System.syncShutdown": 1, "Dense.enabled": 0, "GlobalBA.final": OFF},
}
EXPECT.update(arm_expectations("euroc_p", COMMON_EUROC))
EXPECT.update(arm_expectations("tum_p", COMMON_TUM))
EXPECT.update(arm_expectations("kitti_p", COMMON_KITTI))


def rpath(p):
    p = os.path.expanduser(str(p))
    return p if os.path.isabs(p) else os.path.join(REPO, p)


def strip_comment(line):
    """Drop a trailing '# ...' that is not inside a quoted string."""
    quote = None
    for i, ch in enumerate(line):
        if quote:
            if ch == quote:
                quote = None
        elif ch in "\"'":
            quote = ch
        elif ch == "#":
            return line[:i]
    return line


def norm_value(v):
    v = " ".join(v.split())
    if len(v) >= 2 and v[0] == v[-1] and v[0] in "\"'":
        return v[1:-1]
    try:
        return float(v)
    except ValueError:
        return v


def parse_settings(text):
    """Flat map key -> normalised value; raises ValueError on a duplicate key."""
    out, key, buf = {}, None, []

    def flush():
        if key is not None:
            out[key] = norm_value(" ".join(buf))

    for raw in text.splitlines():
        line = strip_comment(raw).rstrip()
        if not line.strip() or line.startswith("%") or line.strip() == "---":
            continue
        if line[0] in " \t":                      # continuation of a multi-line value
            if key is not None:
                buf.append(line.strip())
            continue
        m = re.match(r"^([^:\s][^:]*?)\s*:(?:\s+(.*))?$", line)
        if not m:
            raise ValueError("cannot parse line %r" % raw)
        flush()
        key, buf = m.group(1), [m.group(2) or ""]
        if key in out:
            raise ValueError("duplicate key %s" % key)
        out[key] = None
    flush()
    return out


def load_settings(path):
    with open(path) as fh:
        return parse_settings(fh.read())


def allowed(key, rules):
    return any(key == r or (r.endswith(".*") and key.startswith(r[:-1])) for r in rules)


def diff_settings(a, b):
    """Keys that differ between settings a (the arm) and b (the reference): {key: (kind, a, b)}."""
    out = {}
    for k in sorted(set(a) | set(b)):
        if k not in b:
            out[k] = ("only in arm", a[k], None)
        elif k not in a:
            out[k] = ("only in reference", None, b[k])
        elif a[k] != b[k]:
            out[k] = ("changed", a[k], b[k])
    return out


def meets(value, want):
    if want is ABSENT:
        return value is None
    if want is OFF:
        return value is None or value == 0
    if isinstance(want, (int, float)) and not isinstance(want, bool):
        return value is not None and not isinstance(value, str) and float(value) == float(want)
    return value == want


def fmt(v):
    if v is None:
        return "-"
    if v == "":
        return '""'
    if isinstance(v, float) and v == int(v):
        return str(int(v))
    return str(v)


def settings_of(reg, arm):
    """{settings path: [sequences using it]} of an arm."""
    cfg = reg["configs"][arm]
    out = {}
    for name, seq in reg["sequences"][arm].items():
        out.setdefault(rpath(seq.get("settings", cfg.get("settings"))), []).append(name)
    return out


def check_pair(reg, arm, ref, rules, cache, verbose=True):
    """Returns the number of problems found for this pair."""
    problems = 0
    pa, pr = settings_of(reg, arm), settings_of(reg, ref)
    seqs_a, seqs_r = {s for v in pa.values() for s in v}, {s for v in pr.values() for s in v}
    pairs = set()
    for s in sorted(seqs_a & seqs_r):
        fa = next(p for p, v in pa.items() if s in v)
        fr = next(p for p, v in pr.items() if s in v)
        pairs.add((fa, fr))
    for fa, fr in sorted(pairs):
        for p in (fa, fr):
            if p not in cache:
                try:
                    cache[p] = load_settings(p)
                except (OSError, ValueError) as exc:
                    print("  ERROR %s: %s" % (os.path.relpath(p, REPO), exc))
                    problems += 1
                    cache[p] = None
        if cache[fa] is None or cache[fr] is None:
            continue
        d = diff_settings(cache[fa], cache[fr])
        rel = lambda p: os.path.relpath(p, REPO)
        print("%s vs %s   (%s vs %s)" % (arm, ref, rel(fa), rel(fr)))
        if fa == fr:
            print("  same file: no difference by construction")
        for k, (kind, va, vr) in d.items():
            ok = allowed(k, rules)
            print("  %-4s %-30s %-17s arm=%s ref=%s" % ("ok" if ok else "BAD", k, kind, fmt(va), fmt(vr)))
            problems += not ok
        if not d and fa != fr:
            print("  no difference in any key (the files differ only in comments or layout)")
        print("  intended: %s -> %s" % (", ".join(rules) if rules else "no difference",
                                        "satisfied" if not any(not allowed(k, rules) for k in d) else "VIOLATED"))
    if not pairs:
        print("%s vs %s: no common sequence, nothing to compare" % (arm, ref))
        problems += 1
    return problems


def check_expectations(reg, arm, cache):
    """The values the arm's own file(s) must hold, whatever the pair says."""
    problems = 0
    want = EXPECT.get(arm)
    if not want:
        return 0
    for path in sorted(settings_of(reg, arm)):
        if path not in cache:
            cache[path] = load_settings(path)
        s = cache[path]
        bad = [(k, s.get(k), w) for k, w in want.items() if not meets(s.get(k), w)]
        for k, got, w in bad:
            print("  BAD  %s: %s = %s, expected %s" % (os.path.relpath(path, REPO), k, fmt(got),
                                                      "absent or 0" if w is OFF else "absent" if w is ABSENT else fmt(w)))
        problems += len(bad)
        print("  %s %s holds its %d expected values" % ("ok  " if not bad else "BAD ", os.path.relpath(path, REPO), len(want)))
    return problems


def run_self_test():
    failures = []

    def check(name, cond):
        print(("PASS " if cond else "FAIL ") + name)
        if not cond:
            failures.append(name)

    a = parse_settings('%YAML:1.0\n# c\nA.x: 1   # trailing\nA.y: "a # not a comment"\nA.z: [1, 2,\n   3]\nM: !!opencv-matrix\n  rows: 2\n  data: [1,\n         2]\n')
    check("comments, quotes and continuation lines", a["A.x"] == 1.0 and a["A.y"] == "a # not a comment"
          and a["A.z"] == "[1, 2, 3]" and a["M"] == "!!opencv-matrix rows: 2 data: [1, 2]")
    b = parse_settings("A.x: 1.0\nA.y: 'a # not a comment'\n")
    check("1 and 1.0 are the same number; quote style does not matter", diff_settings({"A.x": a["A.x"], "A.y": a["A.y"]}, b) == {})
    d = diff_settings({"k": 1.0, "n": 2.0}, {"k": 2.0, "o": 3.0})
    check("changed / only in arm / only in reference", d["k"][0] == "changed" and d["n"][0] == "only in arm" and d["o"][0] == "only in reference")
    check("prefix rules", allowed("Dense.x", ["Dense.*"]) and not allowed("Dens", ["Dense.*"]) and allowed("G.f", ["G.f"]) and not allowed("G.fx", ["G.f"]))
    check("meets: OFF is absent or 0, numbers compare as numbers", meets(None, OFF) and meets(0.0, OFF) and not meets(1.0, OFF)
          and meets(1000.0, 1000) and not meets(None, 1) and meets("online", "online"))
    try:
        parse_settings("a: 1\na: 2\n")
        check("duplicate key is an error", False)
    except ValueError:
        check("duplicate key is an error", True)
    print("\nself-test: %s" % ("ALL PASSED" if not failures else "FAILED: " + ", ".join(failures)))
    return 0 if not failures else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--registry", help="alternative sequences.yaml")
    ap.add_argument("--pair", action="append", default=[], metavar="ARM:REF[:ALLOWED,..]",
                    help="check this pair instead of the built-in pilot pairs (repeatable; ALLOWED = keys, "
                         "'Dense.*' = prefix; empty = no difference allowed)")
    ap.add_argument("--no-expectations", action="store_true", help="skip the per-arm expected-value checks")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    if a.self_test:
        sys.exit(run_self_test())
    with open(a.registry or os.path.join(HERE, "sequences.yaml")) as fh:
        reg = yaml.safe_load(fh)
    pairs = []
    for spec in a.pair:
        parts = spec.split(":")
        if len(parts) < 2:
            ap.error("--pair needs ARM:REF[:ALLOWED]")
        pairs.append((parts[0], parts[1], [x for x in (parts[2].split(",") if len(parts) > 2 else []) if x]))
    custom = bool(pairs)
    pairs = pairs or PAIRS
    problems = 0
    cache = {}
    for arm, ref, rules in pairs:
        for n in (arm, ref):
            if n not in reg["configs"]:
                sys.exit("unknown config %r in sequences.yaml" % n)
        problems += check_pair(reg, arm, ref, rules, cache)
        print()
    if not custom and not a.no_expectations:
        print("expected values per arm:")
        for arm in EXPECT:
            if arm in reg["configs"]:
                print(" %s" % arm)
                problems += check_expectations(reg, arm, cache)
        print()
    print("check_yaml_pairs: %s" % ("every pair differs only where intended" if not problems else "%d PROBLEM(S)" % problems))
    sys.exit(1 if problems else 0)


if __name__ == "__main__":
    main()
