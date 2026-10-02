#!/usr/bin/env python3
"""Generate the settings files of the extra experiments (evaluation/run_extra.sh) from the shipped ones.

Only NEW files are written; an existing file is left alone unless --force. Each output is its source plus
a few appended lines / one replaced value, so that the arms differ in exactly what they are named for:

  <X>_Sync            X + System.syncShutdown: 1                          (arm "base")
  <X>_Sync_C1         X + syncShutdown + GlobalBA.final: 1                (arm "c1")
  <X>_Dense_Sync      X_Dense + syncShutdown                              (arm "faithful")
  <X>_Dense_Imp_Sync  X_Dense_Imp + syncShutdown (it already has GlobalBA.final: 1)   (arm "imp")
  *_Multi             a dense file with Dense.probabilisticMerge: 1 (thesis sec. 3.5: "turn on for multi-sequence runs")

The EuRoC arms of the confirmatory run (EuRoC_T1000*.yaml, EuRoC_Dense_T1000.yaml, EuRoC_Dense_Imp_T1000.yaml)
already carry syncShutdown and are used as they are, except the *_Multi variants below.

    python3 evaluation/harness/make_extra_settings.py [--check] [--force]
"""

import argparse
import os
import re
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

SYNC = """
#--------------------------------------------------------------------------------------------
# System.syncShutdown: 1 makes System::Shutdown() wait for LocalMapping, LoopClosing and any
# running global BA to finish before it returns, in every arm of the extra experiments, so that the
# trajectory is saved from the same settled map whether or not an arm has dense or final-BA
# work at shutdown. 0 (the default) keeps the original behaviour, in which a plain baseline
# returns at once. The wait is logged as "Shutdown: waited X s".
#--------------------------------------------------------------------------------------------
System.syncShutdown: 1
"""

C1 = """
#--------------------------------------------------------------------------------------------
# C1 (melhorias.md): one global bundle adjustment in System::Shutdown(), before the trajectory
# is saved. Sync + C1 isolates what the final GBA alone gives.
#--------------------------------------------------------------------------------------------
GlobalBA.final: 1
"""

MULTI = """
#--------------------------------------------------------------------------------------------
# Multi-sequence variant (thesis sec. 3.5): Dense.probabilisticMerge is turned on. The sequences of a chain run
# in ONE process (ChangeDataset), so the dense thread keeps its global cloud across them and Dense.loadCloud
# is not needed. Everything else is identical to the single-sequence file this was generated from.
#--------------------------------------------------------------------------------------------
"""


def set_key(text, key, value):
    pat = re.compile(r"^%s\s*:.*$" % re.escape(key), re.M)
    if not pat.search(text):
        raise SystemExit("key %s not found; cannot replace it" % key)
    return pat.sub("%s: %s" % (key, value), text, count=1)


def has_key(text, key):
    return re.search(r"^%s\s*:" % re.escape(key), text, re.M) is not None


def with_sync(text):
    return text if has_key(text, "System.syncShutdown") else text.rstrip("\n") + "\n" + SYNC


def with_c1(text):
    return text if has_key(text, "GlobalBA.final") else text.rstrip("\n") + "\n" + C1


def with_multi(text):
    return set_key(text, "Dense.probabilisticMerge", "1").rstrip("\n") + "\n" + MULTI


def plan():
    """(output, source, transform) triples, paths relative to the repo root."""
    out = []
    for d, stem in (("Examples/RGB-D", "TUM1"), ("Examples/RGB-D", "TUM2"), ("Examples/Stereo", "KITTI04-12")):
        p = lambda n: "%s/%s%s.yaml" % (d, stem, n)
        if stem != "KITTI04-12":                       # KITTI04-12_Sync.yaml ships with the repo
            out.append((p("_Sync"), p(""), with_sync))
        out.append((p("_Sync_C1"), p("_Sync"), with_c1))
        out.append((p("_Dense_Sync"), p("_Dense"), with_sync))
        out.append((p("_Dense_Imp_Sync"), p("_Dense_Imp"), with_sync))
    # multi-sequence chains: TUM2 (fr2_large pair) and EuRoC
    out.append(("Examples/RGB-D/TUM2_Dense_Multi_Sync.yaml", "Examples/RGB-D/TUM2_Dense_Sync.yaml", with_multi))
    out.append(("Examples/RGB-D/TUM2_Dense_Imp_Multi_Sync.yaml", "Examples/RGB-D/TUM2_Dense_Imp_Sync.yaml", with_multi))
    out.append(("Examples/Stereo/EuRoC_Dense_T1000_Multi.yaml", "Examples/Stereo/EuRoC_Dense_T1000.yaml", with_multi))
    out.append(("Examples/Stereo/EuRoC_Dense_Imp_T1000_Multi.yaml", "Examples/Stereo/EuRoC_Dense_Imp_T1000.yaml", with_multi))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="only report which files are missing or differ")
    ap.add_argument("--force", action="store_true", help="overwrite existing outputs")
    a = ap.parse_args()
    bad = 0
    generated = {}
    for dst, src, fn in plan():
        src_path, dst_path = os.path.join(REPO, src), os.path.join(REPO, dst)
        if src in generated:                            # a source that is itself generated here
            text = generated[src]
        elif os.path.exists(src_path):
            with open(src_path) as fh:
                text = fh.read()
        else:
            print("MISSING source %s" % src)
            bad += 1
            continue
        new = fn(text)
        generated[dst] = new
        if os.path.exists(dst_path):
            with open(dst_path) as fh:
                same = fh.read() == new
            print("%-9s %s" % ("same" if same else "DIFFERS", dst))
            if not same and a.force and not a.check:
                with open(dst_path, "w") as fh:
                    fh.write(new)
                print("          rewritten")
            bad += 0 if same else 1
            continue
        if a.check:
            print("MISSING   %s" % dst)
            bad += 1
            continue
        with open(dst_path, "w") as fh:
            fh.write(new)
        print("wrote     %s" % dst)
    return 1 if (a.check and bad) else 0


if __name__ == "__main__":
    sys.exit(main())
