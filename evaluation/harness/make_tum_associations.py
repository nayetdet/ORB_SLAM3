#!/usr/bin/env python3
"""Generate a TUM RGB-D association file (rgb <-> depth) for rgbd_tum.

evaluation/associate.py does this but is Python 2 and does not run here. Needed
for sequences with no ready-made file in Examples/RGB-D/associations/, such as
the fr2_large_no_loop / fr2_large_with_loop pair used in the Zhang 2023 thesis.

    python3 evaluation/harness/make_tum_associations.py \
        ~/Datasets/TUM/rgbd_dataset_freiburg2_large_no_loop \
        > Examples/RGB-D/associations/fr2_large_no_loop.txt
"""

import argparse
import os
import sys


def read_file_list(path):
    entries = []
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line[0] == "#":
                continue
            parts = line.split()
            if len(parts) >= 2:
                entries.append((float(parts[0]), parts[1]))
    return entries


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("sequence", help="TUM RGB-D sequence folder (contains rgb.txt, depth.txt)")
    p.add_argument("--max-diff", type=float, default=0.02,
                   help="max rgb/depth timestamp difference in seconds (default 0.02)")
    a = p.parse_args()

    rgb_path = os.path.join(a.sequence, "rgb.txt")
    depth_path = os.path.join(a.sequence, "depth.txt")
    for f in (rgb_path, depth_path):
        if not os.path.exists(f):
            sys.exit("not found: %s" % f)

    rgb = read_file_list(rgb_path)
    depth = read_file_list(depth_path)
    depth_stamps = [d[0] for d in depth]
    used = [False] * len(depth)

    import bisect
    n = 0
    for t_rgb, f_rgb in rgb:
        j = bisect.bisect_left(depth_stamps, t_rgb)
        best, best_d = -1, float("inf")
        for cand in (j - 1, j, j + 1):
            if 0 <= cand < len(depth) and not used[cand]:
                d = abs(depth_stamps[cand] - t_rgb)
                if d < best_d:
                    best, best_d = cand, d
        if best >= 0 and best_d <= a.max_diff:
            used[best] = True
            print("%.6f %s %.6f %s" % (t_rgb, f_rgb, depth[best][0], depth[best][1]))
            n += 1
    print("associated %d of %d rgb frames" % (n, len(rgb)), file=sys.stderr)


if __name__ == "__main__":
    main()
