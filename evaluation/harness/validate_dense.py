#!/usr/bin/env python3
"""Sanity-check the dense reconstruction output of a run.

Reads the .pcd and .ot the Dense Reconstruction thread writes and checks the
things that a sign error, a swapped axis or a bad intrinsic would break. Needs
no PCL: the PCD reader below handles the ascii and binary XYZRGB layouts the
system writes, and the .ot header is plain text.

    python3 evaluation/harness/validate_dense.py \
        --pcd tum_fr1_cloud.pcd --ot tum_fr1_octomap.ot \
        --trajectory CameraTrajectory.txt --traj-format orb_tum
"""

import argparse
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import traj_io  # noqa: E402

FAILED, WARNED = [], []


def check(name, ok, detail="", warn_only=False):
    tag = "PASS" if ok else ("WARN" if warn_only else "FAIL")
    print("  [%s] %s%s" % (tag, name, ("  -- " + detail) if detail else ""))
    if not ok:
        (WARNED if warn_only else FAILED).append(name)


def read_pcd(path):
    """Minimal PCD reader: returns (xyz Nx3 float, rgb Nx3 uint8 or None)."""
    with open(path, "rb") as fh:
        header, fields, size, typ, count, width, height, points, data_fmt = {}, None, None, None, None, 0, 0, None, None
        while True:
            line = fh.readline()
            if not line:
                raise ValueError("%s: truncated header" % path)
            txt = line.decode("ascii", "replace").strip()
            if not txt or txt.startswith("#"):
                continue
            key, _, rest = txt.partition(" ")
            key = key.upper()
            header[key] = rest
            if key == "DATA":
                data_fmt = rest.strip().lower()
                break
        fields = header.get("FIELDS", "").split()
        size = [int(x) for x in header.get("SIZE", "").split()]
        typ = header.get("TYPE", "").split()
        count = [int(x) for x in header.get("COUNT", "1 " * len(fields)).split()]
        npts = int(header.get("POINTS", header.get("WIDTH", "0")))

        if data_fmt == "ascii":
            rows = []
            for line in fh:
                parts = line.split()
                if len(parts) >= len(fields):
                    rows.append([float(p) for p in parts[:len(fields)]])
            arr = np.asarray(rows, dtype=np.float64)
            get = lambda n: arr[:, fields.index(n)] if n in fields else None
            xyz = np.column_stack([get("x"), get("y"), get("z")])
            rgb = None
            if "rgb" in fields:
                packed = get("rgb").astype(np.float32).view(np.uint32)
                rgb = np.column_stack([(packed >> 16) & 255, (packed >> 8) & 255, packed & 255]).astype(np.uint8)
            return xyz, rgb

        if data_fmt not in ("binary", "binary_compressed"):
            raise ValueError("%s: unsupported DATA %r" % (path, data_fmt))
        if data_fmt == "binary_compressed":
            raise ValueError("%s: binary_compressed is not supported; the system writes "
                             "plain binary via savePCDFileBinary" % path)

        np_type = {"F": "f", "U": "u", "I": "i"}
        dtype = np.dtype([(f, np_type[t] + str(s)) for f, t, s in zip(fields, typ, size)])
        raw = fh.read(npts * dtype.itemsize)
        arr = np.frombuffer(raw, dtype=dtype, count=min(npts, len(raw) // dtype.itemsize))
        xyz = np.column_stack([arr["x"], arr["y"], arr["z"]]).astype(np.float64)
        rgb = None
        if "rgb" in fields:
            packed = arr["rgb"].astype(np.float32).view(np.uint32)
            rgb = np.column_stack([(packed >> 16) & 255, (packed >> 8) & 255, packed & 255]).astype(np.uint8)
        return xyz, rgb


def read_ot_header(path):
    """Octomap .ot files start with a plain-text header ending at 'data'."""
    info = {}
    with open(path, "rb") as fh:
        for _ in range(32):
            line = fh.readline()
            if not line:
                break
            txt = line.decode("ascii", "replace").strip()
            if txt.startswith("data"):
                break
            if txt.startswith("#") or not txt:
                continue
            key, _, rest = txt.partition(" ")
            info[key] = rest.strip()
    return info


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--pcd", required=True)
    p.add_argument("--ot")
    p.add_argument("--trajectory", help="estimated trajectory of the same run")
    p.add_argument("--traj-format", default="orb_tum", choices=sorted(traj_io.LOADERS))
    p.add_argument("--expect-extent", type=float, default=None,
                   help="rough scene size in m; warns if the cloud is far off")
    a = p.parse_args()

    print("== point cloud ==")
    xyz, rgb = read_pcd(a.pcd)
    n = len(xyz)
    check("cloud is not empty", n > 0, "%d points" % n)
    if n == 0:
        print("\nNothing else can be checked."); return 1

    finite = np.isfinite(xyz).all(axis=1)
    check("no NaN / inf coordinates", finite.all(),
          "%d bad of %d" % ((~finite).sum(), n))
    xyz = xyz[finite]

    lo, hi = xyz.min(axis=0), xyz.max(axis=0)
    extent = hi - lo
    print("       bbox  x[%.2f, %.2f]  y[%.2f, %.2f]  z[%.2f, %.2f]"
          % (lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
    print("       size  %.2f x %.2f x %.2f m" % tuple(extent))

    check("extent is non-degenerate on all three axes", (extent > 1e-3).all(),
          "a flat or collinear cloud means the back-projection collapsed an axis")
    check("cloud is not a single point", float(np.linalg.norm(extent)) > 0.05)

    at_origin = (np.abs(xyz) < 1e-6).all(axis=1).sum()
    check("points are not piled at the origin", at_origin < max(10, 0.01 * n),
          "%d exactly at (0,0,0)" % at_origin)

    # A cloud that is essentially planar usually means one coordinate was derived
    # wrongly (e.g. a constant depth, or x/y swapped with a constant).
    centred = xyz - xyz.mean(axis=0)
    sv = np.linalg.svd(centred, compute_uv=False) / np.sqrt(max(len(xyz) - 1, 1))
    ratio = float(sv[2] / sv[0]) if sv[0] > 0 else 0.0
    check("cloud has real 3D spread", ratio > 1e-3,
          "smallest/largest principal spread = %.4g (sv=%.3f/%.3f/%.3f)"
          % (ratio, sv[0], sv[1], sv[2]))

    if a.expect_extent:
        got = float(np.max(extent))
        ok = 0.2 * a.expect_extent <= got <= 5.0 * a.expect_extent
        check("scene size is in the expected ballpark", ok,
              "largest extent %.2f m vs expected ~%.2f m" % (got, a.expect_extent),
              warn_only=True)

    if rgb is not None:
        uniq = len(np.unique(rgb.view(np.void(rgb.dtype.itemsize * 3).__class__)
                             if False else rgb[:, 0] * 65536 + rgb[:, 1] * 256 + rgb[:, 2]))
        check("colour channel carries more than one value", uniq > 1,
              "%d distinct colours" % uniq, warn_only=True)

    if a.trajectory:
        print("== agreement with the trajectory of the same run ==")
        _, poses = traj_io.load(a.trajectory, a.traj_format)
        t = poses[:, :3, 3]
        tlo, thi = t.min(axis=0), t.max(axis=0)
        print("       trajectory bbox  %.2f x %.2f x %.2f m" % tuple(thi - tlo))
        # The map is built from what the camera saw, so it must surround the path.
        overlap = np.all(lo <= thi + 1e-6) and np.all(hi >= tlo - 1e-6)
        check("cloud bounding box overlaps the trajectory", overlap,
              "no overlap means the cloud is in the wrong frame -- check Twc")
        margin = np.minimum(thi - lo, hi - tlo).min()
        check("cloud extends around the path, not just along it", margin > -1e-6,
              "tightest margin %.2f m" % margin, warn_only=True)
        centroid_gap = float(np.linalg.norm(xyz.mean(axis=0) - t.mean(axis=0)))
        scene = float(np.max(np.maximum(extent, thi - tlo)))
        check("cloud centroid is near the travelled volume", centroid_gap < 2.0 * scene,
              "%.2f m apart, scene ~%.2f m" % (centroid_gap, scene), warn_only=True)

    if a.ot and os.path.exists(a.ot):
        print("== octomap ==")
        info = read_ot_header(a.ot)
        print("       header: %s" % ", ".join("%s=%s" % kv for kv in info.items()))
        check("octree has nodes", int(info.get("size", "0")) > 0, "size=%s" % info.get("size"))
        check("octree id is a colour tree", "Color" in info.get("id", ""),
              "id=%s" % info.get("id"), warn_only=True)
        pcd_mb = os.path.getsize(a.pcd) / 1e6
        ot_mb = os.path.getsize(a.ot) / 1e6
        ratio_io = pcd_mb / ot_mb if ot_mb > 0 else 0.0
        print("       file sizes: pcd %.2f MB, ot %.2f MB -> compression %.2fx" % (pcd_mb, ot_mb, ratio_io))
        # Thesis Table IX reports ~4.8-6.5x across the three datasets.
        check("compression ratio is in the range Table IX reports", 2.0 <= ratio_io <= 12.0,
              "%.2fx (thesis: 4.77-6.55x)" % ratio_io, warn_only=True)

    print("")
    if FAILED:
        print("FAILED: %d check(s): %s" % (len(FAILED), ", ".join(FAILED)))
        if WARNED: print("warnings: %s" % ", ".join(WARNED))
        return 1
    if WARNED:
        print("passed with %d warning(s): %s" % (len(WARNED), ", ".join(WARNED)))
        return 0
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
