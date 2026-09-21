# Reproduction: Zhang 2023, Dense Reconstruction with Probabilistic Multi-Sequence Merging

Reproduces the system of Hanxiang Zhang, *"Dense Reconstruction from Visual SLAM with
Probabilistic Multi-Sequence Merging"*, MASc thesis, Dalhousie University, December 2023,
on top of stock ORB-SLAM3 v1.0.

Everything is **off by default**: with `Dense.enabled: 0` no dense thread is created and
the system is stock ORB-SLAM3. That matters, because the baseline and the reproduction have
to be measurable against each other.

## What is implemented

| Thesis | Where |
|---|---|
| Fig. 12, Dense Reconstruction thread | `src/PointCloudMapping.cc`, fed from `Tracking::CreateNewKeyFrame` |
| Algorithm 1 (Fig. 13), RGB-D point cloud | `Impl::GenerateCloudRGBD` |
| Algorithm 2 (Fig. 15), stereo point cloud | `Impl::GenerateCloudStereo` |
| Sec. 3.2, voxel filter (">90% of voxels removed") | `Impl::Run`, per keyframe |
| Sec. 3.4, Octomap conversion | `Impl::InsertIntoOctomap` |
| Eq. (34), depth→confidence sigmoid | `Impl::Confidence` |
| Eq. (35), probabilistic merge | `Impl::MergeProbabilistic` |
| Sec. 3.5, multi-sequence reuse | `Dense.loadCloud` + ORB-SLAM3's native Atlas save/load |
| Table X, Dense Reconstruction timings | `PointCloudMapping::PrintTimingSummary` |

Outputs `<savePrefix>_cloud.pcd` and `<savePrefix>_octomap.ot`, the two files Tables VIII
and IX compare.

## Deviations from the thesis, and why

These are judgment calls. They are listed so the reproduction can be defended or corrected,
not buried.

**1. Pixel indexing in Algorithms 1 and 2.** Lines 14–15 (and 18–19) read
`x ← (row − cx)·z/fx`, `y ← (col − cy)·z/fy`, driving the *horizontal* coordinate from the
*row* index. Taken literally this transposes the cloud. The standard pinhole
back-projection is used instead — column is the horizontal pixel *u*, row the vertical
pixel *v* — which is what the thesis's own eq. (29) and Figure 14 describe. Believed to be
a notation slip in the write-up.

**2. Rectification (Algorithm 2 line 4) is not re-done.** ORB-SLAM3 already rectifies the
stereo pair in `System::TrackStereo` (`src/System.cc`) before `Tracking` ever sees it.
Rectifying twice would be wrong. The ROI of line 13 is kept, as configurable margins plus
an automatic left margin of `sgbmNumDisparities` where SGBM cannot produce a disparity.

**3. Disparity (line 11) uses OpenCV SGBM.** The thesis says only that OpenCV is used for
this step and does not name the matcher or its parameters.

**4. The global voxel filter runs only when the probabilistic merge is off.** Sec. 3.5 ends
by stating the merge itself removes redundant points, so applying both would double-filter.
The per-keyframe filter of sec. 3.2 always runs.

**5. The merge searches a per-keyframe snapshot of the global cloud.** Sec. 3.5 says the
search-and-replace is "iterated as many times as necessary". Rebuilding the KD-tree after
every single merged point is quadratic and would not finish; the tree is therefore built
once per keyframe. Points added by the *same* keyframe are not re-searched, which costs
nothing in practice because the per-keyframe voxel filter has already spaced them at least
`resolution` apart. Capped at 8 iterations per point.

**6. Confidence of a prior cloud.** Points loaded from a previous sequence carry no recorded
depth, so they are seeded at `d0`, i.e. confidence 0.5 — a new observation is then weighted
purely by its own distance.

## Finding: the compression ratio of Table IX is mostly file encoding

Measured on TUM fr1_desk with the shipped settings: 1,163,624 points, octomap at
0.01 m, 1,444,254 nodes.

| | size | ratio vs .ot |
|---|---|---|
| `.pcd` binary (`savePCDFileBinary`, 16 B/point) | 18.62 MB | **1.61x** |
| `.pcd` ASCII (~40 B/point) | 48.73 MB | **4.22x** |
| `.ot` ColorOcTree | 11.55 MB | — |
| *thesis, Table IX* | *36.7 MB / 7.5 MB* | *4.89x* |

The thesis never states the PCD encoding. Its reported 4.77-6.55x only reproduces if the
dense map was written as ASCII text and compared against a binary octree. Against a binary
`.pcd` the same data gives 1.6x. So "the size of each dense point cloud map is reduced to
approximately one-fifth after the conversion" (abstract) measures text-vs-binary encoding
at least as much as Octree efficiency.

This system writes binary, which is why `validate_dense.py` expects the lower figure.

## Finding: Octomap ray casting cannot be what the thesis did

`insertPointCloud` carves free space along every ray, which is what gives eq. (31)-(33)'s
log-odds update something to decrease. It also creates a node per voxel per ray: on
fr1_desk that produced 4,868,604 nodes and a **38.95 MB** `.ot`, i.e. the octomap came out
*larger* than the point cloud (0.50x), and cost **1979 ms per keyframe**.

Inserting only the occupied endpoints gives 1,444,254 nodes, 11.55 MB and **16.7 ms** per
keyframe — 119x faster. Since Table IX reports the octomap as smaller, the thesis cannot
have been carving free space, despite sec. 3.4 describing the full occupancy update.
`Dense.octomapRayCast` selects between the two and defaults to off.

Note the consequence: the resulting map records only occupied space, so it does not
distinguish "free" from "unknown". That is fine for the file-size comparison the thesis
makes, but not for the navigation use case its sec. 3.4 motivates.

## Parameters the thesis does not publish

`d_min`, `d_max`, `P_max`, `P_min`, the voxel resolution, the merge distance and the SGBM
settings are never given. The values in the shipped `*_Dense.yaml` files are documented
choices following the thesis's qualitative guidance (sec. 4.4: ~0.01 m leaf indoors, ≥0.05 m
outdoors with 0.2 m typical for KITTI). **Any quantitative comparison against Tables V–IX
should say this.** They are all in the settings file, so a run is reproducible from that
file alone.

## Building

Needs PCL and octomap on top of the usual dependencies; both are in the devcontainer and in
`shell.nix`. Without them CMake prints `Dense reconstruction: DISABLED` and the feature
compiles to no-ops — the rest of ORB-SLAM3 is unaffected.

```bash
./build.sh
# look for: -- Dense reconstruction: ENABLED (PCL 1.15.0, octomap 1.9.7)
```

Three upstream fixes were needed to build at all on a current toolchain:

- **C++17** instead of C++11 (`CMakeLists.txt`). Pangolin ≥ 0.8 ships sigslot, whose headers
  use `std::decay_t`; every translation unit reaching `include/Map.h` (which includes
  `pangolin.h`) failed to compile as C++11. PCL ≥ 1.13 also requires C++17. This is a
  pre-existing blocker, unrelated to the dense code.
- **`LoopClosing::mnFullBAIdx` was `bool`, now `int`** (`include/LoopClosing.h`). It is a
  generation counter: `RunGlobalBundleAdjustment` snapshots it and re-checks it to notice
  that a newer loop aborted its optimisation. As a `bool` it saturated at `true` on the
  first increment, so every abort after the first went undetected. C++17 forbids `++` on
  `bool`, which is how it surfaced. **This changes runtime behaviour** and should be noted
  when comparing against numbers produced by unpatched ORB-SLAM3.
- **Sophus tests/examples disabled** (`build.sh`). They fail under GCC 13+ and ORB-SLAM3
  uses Sophus header-only anyway.

## Running

```bash
# baseline (stock) and reproduction, same binary, settings differ only in Dense.enabled
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt \
    Examples/Stereo/EuRoC_Dense.yaml <seq> \
    Examples/Stereo/EuRoC_TimeStamps/MH01.txt mh01_dense
```

Through the evaluation harness, which is what makes the comparison meaningful:

```bash
python3 evaluation/harness/run_benchmark.py --config euroc_stereo       --runs 10 --tag baseline
python3 evaluation/harness/run_benchmark.py --config euroc_stereo_dense --runs 10 --tag dense
python3 evaluation/harness/run_benchmark.py --compare evaluation/results/baseline evaluation/results/dense
```

Multi-sequence (sec. 3.5): run the first sequence, then point `Dense.loadCloud` at its
`.pcd` and set `Dense.probabilisticMerge: 1` for the next, alongside ORB-SLAM3's own
`System.LoadAtlasFromFile` / `System.SaveAtlasToFile` for the sparse map.

## Known limitations

- The dense thread has an **unbounded queue**. That is faithful to the thesis — and it is
  exactly the coupling that costs accuracy, since a backed-up queue starves Local BA
  (`LocalMapping::InsertKeyFrame` sets `mbAbortBA`, which reaches g2o through
  `setForceStopFlag`). Bounding it is an *improvement*, deliberately not done here.
- Octomap `insertPointCloud` ray-casts from the camera centre. This is what makes the
  log-odds update of eq. (31)–(33) meaningful, and it is the dominant cost outdoors.
- Not validated end to end against a real dataset yet: no sequences were available on this
  machine. Settings parsing, construction, threading and shutdown are covered by a smoke
  test; the cloud/Octomap output needs a real run to confirm.
