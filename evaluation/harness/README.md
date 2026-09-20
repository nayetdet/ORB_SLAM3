# Evaluation harness (Fase 0)

A measurement protocol you can trust, built before changing any SLAM code. Without
it there is no way to tell an improvement from run-to-run noise.

Dependencies: Python 3 + numpy + PyYAML. No `evo`, no `scipy`.

## Quick start

```bash
# 1. validate the maths (no dataset needed) -- 17 checks
python3 evaluation/harness/selftest.py

# 2. check every path before committing to a long run
python3 evaluation/harness/run_benchmark.py --config euroc_stereo --dry-run

# 3. measure the baseline on THIS machine
python3 evaluation/harness/run_benchmark.py \
    --config euroc_stereo --sequences MH04,V103 --runs 10 --tag baseline

# 4. measure the change, then compare
python3 evaluation/harness/run_benchmark.py \
    --config euroc_stereo_inertial --sequences MH04,V103 --runs 10 --tag inertial
python3 evaluation/harness/run_benchmark.py \
    --compare evaluation/results/baseline evaluation/results/inertial
```

Single trajectory, outside the runner:

```bash
python3 evaluation/harness/evaluate.py \
  --est f_v103.txt --est-format orb_euroc \
  --gt evaluation/Ground_truth/EuRoC_left_cam/V103_GT.txt --gt-format gt_euroc \
  --align se3
```

## What this fixes, and why

### 1. Median of 10 runs, not mean of 3

ORB-SLAM3 is multi-threaded and non-deterministic. On EuRoC the run-to-run spread
is comparable to the differences the Zhang 2023 thesis reports as results — its
Table VII claims an improvement of 2 mm (MH_02, 0.028 → 0.026 m) from a 3-run mean.
That is not measurable at n=3. The ORB-SLAM3 paper reports the median of 10
executions for exactly this reason.

`run_benchmark.py` defaults to `--runs 10` and reports **median, std, min, max**.
If a change is smaller than the std column, it is not a result.

### 2. SE(3) alignment for stereo and RGB-D — never Sim(3)

`evaluation/evaluate_ate_scale.py` always solves for scale (and is Python 2, so it
does not run on this machine at all). Scale-corrected RMSE **hides scale drift**,
which is a real error mode for stereo, and is not comparable with the published
ORB-SLAM2/3 numbers.

The self-test pins this down: on a trajectory scaled by 1.30, Sim(3) reports
`rmse ≈ 0` while SE(3) reports `0.91 m`. For stereo/RGB-D/inertial, `0.91 m` is
the honest number. Sim(3) is correct only for monocular. The alignment is set per
config in `sequences.yaml` and cannot be silently mismatched.

### 3. Ground-truth frame must match the sensor mode

`System::SaveTrajectoryEuRoC` writes the **IMU/body** pose for inertial sensors
(`GetImuPose()`, `mImuCalib.mTbc`) and the **cam0** pose otherwise — see
`src/System.cc`. That is why the repo ships two ground-truth sets:

| mode | ORB-SLAM3 output frame | ground truth to use |
|---|---|---|
| `stereo_euroc` (visual) | cam0 | `evaluation/Ground_truth/EuRoC_left_cam/*_GT.txt` |
| `stereo_inertial_euroc` | body/IMU | `<seq>/mav0/state_groundtruth_estimate0/data.csv` |

Crossing them adds the fixed camera–IMU extrinsic (~7 cm on EuRoC) to every pose.
Since Fase 1 is precisely a stereo vs stereo-inertial comparison, this mistake
would have produced a fake regression. `sequences.yaml` wires each config to the
right one.

### 4. Quaternion and timestamp conventions

| file | timestamp | quaternion |
|---|---|---|
| `SaveTrajectoryEuRoC` (`f_*.txt`, `kf_*.txt`) | nanoseconds (`1e9*t`) | `qx qy qz qw` |
| `SaveTrajectoryTUM` | seconds | `qx qy qz qw` |
| `SaveTrajectoryKITTI` | none (frame index) | 3×4 row-major matrix |
| EuRoC/ASL ground truth | nanoseconds | `qw qx qy qz` |

The orders are **reversed** between ORB-SLAM3 output and EuRoC ground truth.
`traj_io.py` normalises all of it; `selftest.py` asserts both conventions land on
the same rotation.

### 5. Frame trajectory vs keyframe trajectory, stated explicitly

`f_*.txt` (every tracked frame) and `kf_*.txt` (keyframes only) give different
numbers. `--trajectory {frames,keyframes}` defaults to `frames` and the choice is
recorded in `results.json` and in the report header, so two result sets can never
be compared across different trajectory types by accident.

### 6. KITTI is scored with the KITTI metric

ATE in metres over a 3 km sequence is dominated by one far-end drift and is not
what KITTI is scored on. The harness also reports **t_rel [%]** and
**r_rel [deg/100m]**, reimplementing the official devkit (segments of 100–800 m,
step 10 frames), including its discretisation bias so the numbers stay comparable
with published ones. See `metrics.kitti_relative`.

### 7. Failures are recorded, not silently dropped

A crash, a timeout or lost tracking is logged per run with its return code and log
path, and the report shows `ok/N`. Coverage below 70% of the ground truth raises a
warning: **a low ATE on a partial trajectory is not a good result**, it is a lost
track.

## Files

| file | purpose |
|---|---|
| `selftest.py` | 17 checks on the maths and conventions; needs no dataset |
| `traj_io.py` | loaders for every ORB-SLAM3 output and ground-truth format + association |
| `metrics.py` | Umeyama SE(3)/Sim(3) alignment, ATE, KITTI relative errors, aggregation |
| `evaluate.py` | score one trajectory (replaces `evaluate_ate_scale.py`) |
| `run_benchmark.py` | run N executions over a sequence set, aggregate, compare |
| `make_tum_associations.py` | TUM rgb/depth association file (replaces `associate.py`) |
| `sequences.yaml` | dataset roots, configs, sequences, reference numbers |

Results land in `evaluation/results/<tag>/` as `results.json` + `results.md`, with
per-run trajectories and SLAM logs kept alongside.

## Before the first real run

1. `cd Vocabulary && tar -xf ORBvoc.txt.tar.gz`
2. `./build.sh`
3. Set your dataset paths in `sequences.yaml` under `roots:`
4. `--dry-run` to confirm every path resolves
5. Baseline first, change second. The `reference:` numbers in `sequences.yaml`
   (ORB-SLAM3 paper, thesis Tables V and VI) are printed for context only — they
   were measured on other hardware and are not a baseline.
