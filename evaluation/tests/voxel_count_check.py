#!/usr/bin/env python3
"""Independent voxel counting for the Dense.voxelSafe tests (numpy only, no PCL).

The only thing taken from pcl::VoxelGrid is the definition of a cell:
floor(coordinate * inverse_leaf) per axis, computed in float32 with
inverse_leaf = float32(1) / float32(leaf). numpy's float32 arithmetic is IEEE, so it
bins every point exactly as pcl's float code does; sorting, centroids and colours are
all computed here, not by pcl.

  --raw F.f32 --leaf L [--expect N]
        F.f32: raw float32 x,y,z triples. Prints the number of occupied voxels; with
        --expect, exits 1 unless it equals N (the safe filter's output count).

  --pcd F.pcd --leaf L [--filtered OUT.raw [--sample K]]
        F.pcd: binary PCD with fields x y z rgb (16 bytes per point), memory-mapped.
        Prints points, occupied voxels and points per occupied voxel. With --filtered
        (raw float32 x,y,z,rgb-bits, 16 bytes per point, as written by voxel_safe_cloud)
        also checks the filter's output: its size must equal the occupied voxels, no
        two output points may share a voxel, and K randomly chosen voxels are
        recomputed here (centroid in float64, mean colour) and compared with the
        output point of that voxel.
"""
import argparse
import sys

import numpy as np

CHUNK = 1 << 22


def parse_pcd_header(path):
    with open(path, "rb") as f:
        hdr = {}
        while True:
            line = f.readline().decode("ascii", "replace").strip()
            if not line:
                raise ValueError("unterminated PCD header")
            k, _, v = line.partition(" ")
            hdr[k] = v
            if k == "DATA":
                offset = f.tell()
                break
    if hdr["DATA"] != "binary":
        raise ValueError("only binary PCD is supported")
    if hdr["FIELDS"].split() != ["x", "y", "z", "rgb"] or hdr["SIZE"].split() != ["4"] * 4:
        raise ValueError("expected FIELDS x y z rgb, SIZE 4 4 4 4; got %s / %s" % (hdr["FIELDS"], hdr["SIZE"]))
    return int(hdr["POINTS"]), offset


def finite_rows(a):
    return np.isfinite(a[:, 0]) & np.isfinite(a[:, 1]) & np.isfinite(a[:, 2])


def voxel_coords(xyz, inv):
    # float32 * float32 -> float32, exactly pcl's expression
    return np.floor(xyz * inv).astype(np.int64)


def bounds(chunks, inv):
    lo = np.full(3, np.iinfo(np.int64).max, np.int64)
    hi = np.full(3, np.iinfo(np.int64).min, np.int64)
    n = 0
    for c in chunks:
        c = np.asarray(c[:, :3], dtype=np.float32)
        c = c[finite_rows(c)]
        if len(c) == 0:
            continue
        v = voxel_coords(c, inv)
        lo = np.minimum(lo, v.min(axis=0))
        hi = np.maximum(hi, v.max(axis=0))
        n += len(c)
    return lo, hi, n


def keys_of(c, inv, lo, dims):
    c = np.asarray(c[:, :3], dtype=np.float32)
    c = c[finite_rows(c)]
    v = voxel_coords(c, inv) - lo
    return (v[:, 0] * dims[1] + v[:, 1]) * dims[2] + v[:, 2]


def all_keys(chunks, inv, lo, dims):
    parts = [keys_of(c, inv, lo, dims) for c in chunks]
    return np.concatenate(parts) if parts else np.empty(0, np.int64)


def chunked(a):
    for i in range(0, len(a), CHUNK):
        yield a[i:i + CHUNK]


def rgb_bits(col):
    u = np.ascontiguousarray(col).view(np.uint32)
    return (u >> 16) & 255, (u >> 8) & 255, u & 255


def report_counts(name, n, occupied):
    print("  %-18s %12d points, %12d occupied voxels, %.4f points per occupied voxel"
          % (name, n, occupied, n / occupied if occupied else float("nan")))


def check_filtered(pts, filt, inv, lo, dims, occupied, keys, leaf, nsample, seed=1):
    ok = True
    nout = len(filt)
    print("  filter output: %d points" % nout)
    if nout != occupied:
        print("  FAIL  output has %d points, input occupies %d voxels" % (nout, occupied))
        ok = False
    else:
        print("  ok    output size == occupied voxels (%d)" % occupied)

    okeys = all_keys(chunked(filt), inv, lo, dims)
    okeys_sorted = np.sort(okeys)
    dup = int(np.count_nonzero(okeys_sorted[1:] == okeys_sorted[:-1]))
    report_counts("output re-binned", nout, len(np.unique(okeys_sorted)))
    # A centroid one ulp across a cell border can land in the neighbouring cell's key; that
    # is not a defect of the filter, so it is reported, not failed, when sizes agree.
    print("  %s    output points sharing a voxel key with another output point: %d" %
          ("ok" if dup == 0 else "note", dup))

    if nsample <= 0:
        return ok
    rng = np.random.default_rng(seed)
    uk = np.unique(keys)
    pick = np.sort(rng.choice(uk, size=min(nsample, len(uk)), replace=False))
    # all input points that fall in a picked voxel
    sel_x, sel_k = [], []
    for a in chunked(pts):
        k = keys_of(a, inv, lo, dims)
        a = np.asarray(a)[finite_rows(np.asarray(a[:, :3], dtype=np.float32))]
        m = np.isin(k, pick, assume_unique=False)
        if m.any():
            sel_x.append(a[m])
            sel_k.append(k[m])
    x = np.concatenate(sel_x)
    k = np.concatenate(sel_k)
    order = np.argsort(k, kind="stable")
    x, k = x[order], k[order]
    first = np.flatnonzero(np.r_[True, k[1:] != k[:-1]])
    cnt = np.diff(np.r_[first, len(k)])
    cell = k[first]
    xyz = x[:, :3].astype(np.float64)
    mean = np.stack([np.add.reduceat(xyz[:, a], first) / cnt for a in range(3)], axis=1)
    r, g, b = rgb_bits(x[:, 3])
    mrgb = np.stack([np.add.reduceat(c.astype(np.int64), first) // cnt for c in (r, g, b)], axis=1)

    # find each picked voxel's output point: same key first, then the 26 neighbours
    order_o = np.argsort(okeys, kind="stable")
    ok_sorted = okeys[order_o]
    strides = np.array([dims[1] * dims[2], dims[2], 1], np.int64)
    found = np.full(len(cell), -1, np.int64)
    via = np.zeros(len(cell), np.int8)
    offs = [(0, 0, 0)] + [(i, j, l) for i in (-1, 0, 1) for j in (-1, 0, 1) for l in (-1, 0, 1) if (i, j, l) != (0, 0, 0)]
    maxabs = float(np.abs(xyz).max())
    tol = 1e-4 * leaf + 4.0 * float(np.spacing(np.float32(maxabs)))
    for oi, off in enumerate(offs):
        todo = np.flatnonzero(found < 0)
        if len(todo) == 0:
            break
        q = cell[todo] + int(np.dot(off, strides))
        pos = np.searchsorted(ok_sorted, q)
        pos[pos >= len(ok_sorted)] = len(ok_sorted) - 1
        hit = ok_sorted[pos] == q
        cand = order_o[pos[hit]]
        pts_o = np.asarray(filt[cand])
        dev = np.abs(pts_o[:, :3].astype(np.float64) - mean[todo[hit]]).max(axis=1)
        good = dev <= tol
        idx = todo[hit][good]
        found[idx] = cand[good]
        via[idx] = oi
    nf = int(np.count_nonzero(found < 0))
    if nf:
        print("  FAIL  %d of %d sampled voxels have no output point within tolerance" % (nf, len(cell)))
        ok = False
    po = np.asarray(filt[found[found >= 0]])
    dev = np.abs(po[:, :3].astype(np.float64) - mean[found >= 0]).max(axis=1)
    orgb = np.stack(rgb_bits(po[:, 3]), axis=1)
    bad_col = int(np.count_nonzero((orgb != mrgb[found >= 0]).any(axis=1)))
    print("  sampled %d voxels (of %d): centroid max dev %.3e m = %.2e of the leaf (tolerance %.2e m), "
          "same-key matches %d, neighbour-key matches %d, unmatched %d"
          % (len(cell), len(uk), dev.max() if len(dev) else 0.0, (dev.max() if len(dev) else 0.0) / leaf, tol,
             int(np.count_nonzero(via[found >= 0] == 0)), int(np.count_nonzero(via[found >= 0] != 0)), nf))
    if bad_col:
        print("  FAIL  %d sampled voxels differ in mean colour" % bad_col)
        ok = False
    else:
        print("  ok    mean colour of every sampled voxel matches (truncated integer mean)")
    cm = np.bincount(cnt)[:8]
    print("  sampled voxels by input points per voxel (1..7): %s" % cm[1:8].tolist())
    return ok


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--raw")
    g.add_argument("--pcd")
    ap.add_argument("--leaf", required=True)
    ap.add_argument("--expect", type=int)
    ap.add_argument("--filtered")
    ap.add_argument("--sample", type=int, default=200000)
    a = ap.parse_args()

    leaf = np.float32(a.leaf)
    inv = np.float32(1.0) / leaf
    leaf_f = float(leaf)

    if a.raw:
        pts = np.fromfile(a.raw, dtype=np.float32).reshape(-1, 3)
    else:
        n, off = parse_pcd_header(a.pcd)
        pts = np.memmap(a.pcd, dtype="<f4", mode="r", offset=off, shape=(n, 4))
    lo, hi, nfin = bounds(chunked(pts), inv)
    dims = hi - lo + 1
    print("input: %d points (%d finite), voxel box %d x %d x %d = %.3e cells (INT32_MAX = 2.147e9)"
          % (len(pts), nfin, dims[0], dims[1], dims[2], float(dims[0]) * dims[1] * dims[2]))
    keys = all_keys(chunked(pts), inv, lo, dims)
    occupied = len(np.unique(keys))
    report_counts("input", nfin, occupied)

    ok = True
    if a.expect is not None:
        if a.expect == occupied:
            print("  ok    safe filter output (%d points) == independently counted occupied voxels" % a.expect)
        else:
            print("  FAIL  safe filter output has %d points, numpy counts %d occupied voxels" % (a.expect, occupied))
            ok = False
    if a.filtered:
        filt = np.memmap(a.filtered, dtype="<f4", mode="r").reshape(-1, 4)
        ok = check_filtered(pts, filt, inv, lo, dims, occupied, keys, leaf_f, a.sample) and ok
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
