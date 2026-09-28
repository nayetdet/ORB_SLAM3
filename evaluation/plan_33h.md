# Plan: full 10-run protocol (baseline, dense-faithful, dense-imp, offline)

Branch `melhorias`. Every number below is an ESTIMATE, not a measurement of the full protocol; read the assumptions first.

## Assumptions behind the estimates

- **No validation report exists in the repo** (`docs/Validacao_Pendente.md`, referenced by `melhorias.md`, is absent). Timings are therefore derived from what IS measured or documented:
  - measured here on TUM fr1_desk (23 s sequence, `melhorias` build, container, 12 cores): `Imp` run 43 s wall, `Offline` run 58 s wall; offline rebuild of 127 keyframes took 4.03 s (32 ms/keyframe); 127-143 keyframes per run (about 5.5 keyframes/s); online dense 15-36 ms/keyframe (`docs/Reproduction_Zhang2023.md`: octomap 16.7 ms/keyframe).
  - `rgbd_tum`, `stereo_euroc`, `stereo_kitti` sleep to the dataset timestamps, so wall time is about the sequence duration plus a fixed overhead. Sequence durations are the public dataset durations (EuRoC about 100-180 s, KITTI frames/10 Hz, TUM per the dataset page); they were not re-measured.
  - Fixed overhead per run: 15 s baseline (vocabulary load, init, shutdown, trajectory save); +7 s dense-faithful (queue drain, PCD/octomap write); +10 s dense-imp (drain plus final global BA); +3 s plus rebuild for offline.
  - Offline rebuild per keyframe: 32 ms RGB-D (measured); **150 ms stereo (ASSUMED, SGBM 96 disparities, not measured)**. Keyframe rate: 5.5/s TUM (measured), 3/s EuRoC and 2/s KITTI (assumed).
  - +4 s per run for harness scoring. `validate_dense.py`, disk cleanup and dataset loading effects are not included; add 5-10% slack.
- **Only TUM data is present locally** (`~/Datasets/TUM`: fr1_desk, fr1_room, fr2_desk, fr3_office, fr2_large_with_loop). `~/Datasets/EuRoC` is empty and no KITTI is installed, so the EuRoC and KITTI blocks cannot run until those are downloaded.
- Sequence subset (chosen to fit about 33 h, includes the loop sequences the spec requires): TUM fr1_desk, fr1_room, fr2_desk, fr3_office, fr2_large_with_loop; EuRoC MH01, MH02, MH04, MH05, V101, V102, V103; KITTI 00, 03, 05, 07, 09. Add sequences by extending `--sequences`; each extra sequence costs (sum of the four per-run times) x 10.

- Confounds to report, not hide: `dense-imp` includes C1 (final global BA), which also improves a plain baseline; a baseline+C1 arm is NOT part of this plan (no settings file exists for it). `offline` deliberately has no C1, so its trajectory should match `baseline` up to run-to-run noise.

## 0. Setup (one time, about 10-15 min, not in the total)

Build in a copy so `git status` of the repo stays clean (the repo `.gitignore` does not ignore `.so`/`.log`/`.pcd` files inside build directories):

```bash
cd /home/gabrielrichard/code/ORB_SLAM3 && git checkout melhorias
export WORK=$HOME/orb_work && mkdir -p $WORK
rsync -a --delete --exclude .git --exclude .claude --exclude evaluation/results ./ $WORK/
tar -xzf $WORK/Vocabulary/ORBvoc.txt.tar.gz -C $WORK/Vocabulary
docker run --rm -u $(id -u):$(id -g) -e HOME=/home/vscode -v "$WORK":/workspaces/ORB_SLAM3 -v "$HOME/Datasets":/home/vscode/Datasets:ro -w /workspaces/ORB_SLAM3 orb_slam3:dense bash -c '\
  set -e; for d in DBoW2 g2o; do (cd Thirdparty/$d && mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j12); done; \
  mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DDENSE_RECONSTRUCTION=ON && make -j12'
# Sanity: selftest, then a 1-run smoke test per arm on the shortest sequence
docker run --rm -u $(id -u):$(id -g) -e HOME=/home/vscode -v "$WORK":/workspaces/ORB_SLAM3 -v "$HOME/Datasets":/home/vscode/Datasets:ro -w /workspaces/ORB_SLAM3 orb_slam3:dense python3 evaluation/harness/selftest.py
python3 $WORK/evaluation/harness/make_tum_associations.py $HOME/Datasets/TUM/rgbd_dataset_freiburg2_large_with_loop > $WORK/Examples/RGB-D/associations/fr2_large_with_loop.txt
```

Runtime library path: if the binaries do not find `libORB_SLAM3.so`, `libDBoW2.so`, `libg2o.so`, add `-e LD_LIBRARY_PATH=/workspaces/ORB_SLAM3/lib:/workspaces/ORB_SLAM3/Thirdparty/DBoW2/lib:/workspaces/ORB_SLAM3/Thirdparty/g2o/lib` to `docker run` (this was needed for the smoke test).

Rules for a valid comparison: nothing else running on the machine; viewer off (already `System.UseViewer: 0`); do not change CPU governor between blocks; results go outside git (`--out $WORK/results`); KITTI offline uses `/tmp/orbslam3_dense_spill` (disk); keep about 40 GB free (dense runs write about 40 MB each, 10 runs per sequence).

## 1. Ordered commands

Order: dataset by dataset (TUM, EuRoC, KITTI), and within a dataset the four arms back to back, so every arm sees the same thermal and background conditions. `D` below abbreviates the docker prefix from section 0.

```bash
D='docker run --rm -u $(id -u):$(id -g) -e HOME=/home/vscode -v "$WORK":/workspaces/ORB_SLAM3 -v "$HOME/Datasets":/home/vscode/Datasets:ro -w /workspaces/ORB_SLAM3 orb_slam3:dense'
```

| # | Dataset | Arm | Config | Runs | Est. time |
|---|---|---|---|---|---|
| 1 | tum | baseline | `tum_rgbd` | 5 seq x 10 | 1.46 h |
| 2 | tum | dense-faithful | `tum_rgbd_dense` | 5 seq x 10 | 1.56 h |
| 3 | tum | dense-imp | `tum_rgbd_dense_imp` | 5 seq x 10 | 1.60 h |
| 4 | tum | offline | `tum_rgbd_dense_offline` | 5 seq x 10 | 1.71 h |
| 5 | euroc | baseline | `euroc_stereo` | 7 seq x 10 | 2.80 h |
| 6 | euroc | dense-faithful | `euroc_stereo_dense` | 7 seq x 10 | 2.94 h |
| 7 | euroc | dense-imp | `euroc_stereo_dense_imp` | 7 seq x 10 | 2.99 h |
| 8 | euroc | offline | `euroc_stereo_dense_offline` | 7 seq x 10 | 3.95 h |
| 9 | kitti | baseline | `kitti_stereo` | 5 seq x 10 | 3.26 h |
| 10 | kitti | dense-faithful | `kitti_stereo_dense` | 5 seq x 10 | 3.36 h |
| 11 | kitti | dense-imp | `kitti_stereo_dense_imp` | 5 seq x 10 | 3.40 h |
| 12 | kitti | offline | `kitti_stereo_dense_offline` | 5 seq x 10 | 4.20 h |

```bash
# 1. tum / baseline  (est. 1.46 h, cumulative 1.46 h)
$D python3 evaluation/harness/run_benchmark.py --config tum_rgbd --sequences fr1_desk,fr1_room,fr2_desk,fr3_office,fr2_large_with_loop --runs 10 --tag tum_baseline --out /workspaces/ORB_SLAM3/results
# 2. tum / dense-faithful  (est. 1.56 h, cumulative 3.02 h)
$D python3 evaluation/harness/run_benchmark.py --config tum_rgbd_dense --sequences fr1_desk,fr1_room,fr2_desk,fr3_office,fr2_large_with_loop --runs 10 --tag tum_dense_faithful --out /workspaces/ORB_SLAM3/results
# 3. tum / dense-imp  (est. 1.60 h, cumulative 4.62 h)
$D python3 evaluation/harness/run_benchmark.py --config tum_rgbd_dense_imp --sequences fr1_desk,fr1_room,fr2_desk,fr3_office,fr2_large_with_loop --runs 10 --tag tum_dense_imp --out /workspaces/ORB_SLAM3/results
# 4. tum / offline  (est. 1.71 h, cumulative 6.33 h)
$D python3 evaluation/harness/run_benchmark.py --config tum_rgbd_dense_offline --sequences fr1_desk,fr1_room,fr2_desk,fr3_office,fr2_large_with_loop --runs 10 --tag tum_offline --out /workspaces/ORB_SLAM3/results
# 5. euroc / baseline  (est. 2.80 h, cumulative 9.13 h)
$D python3 evaluation/harness/run_benchmark.py --config euroc_stereo --sequences MH01,MH02,MH04,MH05,V101,V102,V103 --runs 10 --tag euroc_baseline --out /workspaces/ORB_SLAM3/results
# 6. euroc / dense-faithful  (est. 2.94 h, cumulative 12.07 h)
$D python3 evaluation/harness/run_benchmark.py --config euroc_stereo_dense --sequences MH01,MH02,MH04,MH05,V101,V102,V103 --runs 10 --tag euroc_dense_faithful --out /workspaces/ORB_SLAM3/results
# 7. euroc / dense-imp  (est. 2.99 h, cumulative 15.06 h)
$D python3 evaluation/harness/run_benchmark.py --config euroc_stereo_dense_imp --sequences MH01,MH02,MH04,MH05,V101,V102,V103 --runs 10 --tag euroc_dense_imp --out /workspaces/ORB_SLAM3/results
# 8. euroc / offline  (est. 3.95 h, cumulative 19.02 h)
$D python3 evaluation/harness/run_benchmark.py --config euroc_stereo_dense_offline --sequences MH01,MH02,MH04,MH05,V101,V102,V103 --runs 10 --tag euroc_offline --out /workspaces/ORB_SLAM3/results
# 9. kitti / baseline  (est. 3.26 h, cumulative 22.28 h)
$D python3 evaluation/harness/run_benchmark.py --config kitti_stereo --sequences 00,03,05,07,09 --runs 10 --tag kitti_baseline --out /workspaces/ORB_SLAM3/results
# 10. kitti / dense-faithful  (est. 3.36 h, cumulative 25.64 h)
$D python3 evaluation/harness/run_benchmark.py --config kitti_stereo_dense --sequences 00,03,05,07,09 --runs 10 --tag kitti_dense_faithful --out /workspaces/ORB_SLAM3/results
# 11. kitti / dense-imp  (est. 3.40 h, cumulative 29.04 h)
$D python3 evaluation/harness/run_benchmark.py --config kitti_stereo_dense_imp --sequences 00,03,05,07,09 --runs 10 --tag kitti_dense_imp --out /workspaces/ORB_SLAM3/results
# 12. kitti / offline  (est. 4.20 h, cumulative 33.24 h)
$D python3 evaluation/harness/run_benchmark.py --config kitti_stereo_dense_offline --sequences 00,03,05,07,09 --runs 10 --tag kitti_offline --out /workspaces/ORB_SLAM3/results
```

Per-run estimates (seconds, including 4 s scoring):

| Dataset | Sequence | baseline | dense-faithful | dense-imp | offline |
|---|---|---|---|---|---|
| tum | fr1_desk | 42 | 49 | 52 | 49 |
| tum | fr1_room | 68 | 75 | 78 | 79 |
| tum | fr2_desk | 118 | 125 | 128 | 138 |
| tum | fr3_office | 106 | 113 | 116 | 124 |
| tum | fr2_large_with_loop | 192 | 199 | 202 | 225 |
| euroc | MH01 | 201 | 208 | 211 | 285 |
| euroc | MH02 | 169 | 176 | 179 | 239 |
| euroc | MH04 | 118 | 125 | 128 | 165 |
| euroc | MH05 | 130 | 137 | 140 | 182 |
| euroc | V101 | 163 | 170 | 173 | 230 |
| euroc | V102 | 103 | 110 | 113 | 143 |
| euroc | V103 | 124 | 131 | 134 | 174 |
| kitti | 00 | 473 | 480 | 483 | 612 |
| kitti | 03 | 99 | 106 | 109 | 126 |
| kitti | 05 | 295 | 302 | 305 | 380 |
| kitti | 07 | 129 | 136 | 139 | 165 |
| kitti | 09 | 178 | 185 | 188 | 228 |

## 2. Comparisons and dense validation (after the runs; minutes, not in the total)

```bash
$D python3 evaluation/harness/run_benchmark.py --compare results/tum_baseline results/tum_dense_faithful
$D python3 evaluation/harness/run_benchmark.py --compare results/tum_baseline results/tum_dense_imp
$D python3 evaluation/harness/run_benchmark.py --compare results/tum_baseline results/tum_offline
$D python3 evaluation/harness/run_benchmark.py --compare results/euroc_baseline results/euroc_dense_faithful
$D python3 evaluation/harness/run_benchmark.py --compare results/euroc_baseline results/euroc_dense_imp
$D python3 evaluation/harness/run_benchmark.py --compare results/euroc_baseline results/euroc_offline
$D python3 evaluation/harness/run_benchmark.py --compare results/kitti_baseline results/kitti_dense_faithful
$D python3 evaluation/harness/run_benchmark.py --compare results/kitti_baseline results/kitti_dense_imp
$D python3 evaluation/harness/run_benchmark.py --compare results/kitti_baseline results/kitti_offline
# Dense sanity for one run per arm and loop sequence (fr2_large_with_loop, MH05, 00):
$D python3 evaluation/harness/validate_dense.py --pcd results/tum_offline/fr2_large_with_loop/run00/<prefix>_cloud.pcd --ot results/tum_offline/fr2_large_with_loop/run00/<prefix>_octomap.ot --trajectory results/tum_offline/fr2_large_with_loop/run00/CameraTrajectory.txt --traj-format orb_tum
```

`run_benchmark` moves only the trajectory out of each run directory; the `.pcd`/`.ot`/`.log` stay in `results/<tag>/<seq>/runNN/`. Note the runner scores the trajectory only: dense cloud quality (point counts, coverage) needs `validate_dense.py` or a look at the `[Dense]` lines in `slam.log`. Offline Shutdown time (rebuild) is inside the wall-clock reported per run.

## 3. Total

| Arm | Estimated |
|---|---|
| baseline | 7.52 h |
| dense-faithful | 7.85 h |
| dense-imp | 7.99 h |
| offline | 9.87 h |
| **Total (runs only)** | **33.2 h** |
| Slack (+7% for validate_dense, cleanup, restarts) | 2.3 h |
| **Total with slack** | **35.6 h** |

The largest uncertainty is the offline stereo rebuild (150 ms/keyframe assumed): each +50 ms/keyframe adds about 0.7 h over the EuRoC and KITTI offline blocks. Replace it with a measured value from the first EuRoC offline run and recompute.
