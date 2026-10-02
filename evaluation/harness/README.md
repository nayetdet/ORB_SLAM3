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

---

# Interleaved, CPU-limited pilot (tools added after the 39 h benchmark)

The 12-block benchmark ran each arm as one block, on a machine with 12 threads of which it used
2-3, and its baseline returned from `System::Shutdown()` without waiting for the backend. Three
things follow for the next phase: interleave the arms run by run, give the system the CPU budget of
the thesis laptop (2 cores / 4 threads), and make the arms differ only in what is being tested.
Every new C++ switch is opt-in (`System.syncShutdown`, `Dense.voxelSafe`, `GlobalBA.final`, ...);
the default of each reproduces the old behaviour, so the thesis-faithful mode stays reproducible.

| file | purpose |
|---|---|
| `run_interleaved.py` | run several arms (configs of `sequences.yaml`) on the same sequences, interleaved run by run; score each run at once; write `results.json` per arm and `order.json` |
| `log_metrics.py` | author-comparable numbers from a run directory (`slam.log`, `.pcd/.ot/.bt`): tracking time, dense stage times, points, file sizes, dropped keyframes, shutdown wait and give-ups, voxel-filter calls and overflows, refused priority boosts, final GBA, loops, re-inits |
| `stats_compare.py` | now also reads the flat layout `DIR/<arm>/results.json`, extra `--pair`s, `--null-arms` (A/A) and any per-run `--metrics` |
| `check_yaml_pairs.py` | proves from the settings files that each pilot arm differs from its reference only in the intended keys |
| `../run_pilot.sh` | guards, 25 s preflight, then `run_interleaved.py` in the `orb_slam3:dense` image with a CPU limit |
| `../../Examples/Stereo/EuRoC_T1000*.yaml`, `EuRoC_Dense_T1000.yaml`, `EuRoC_Dense_Imp_T1000.yaml`, `KITTI04-12_Sync.yaml` | the pilot settings (`ORBextractor.nFeatures` 1000 like the thesis; `System.syncShutdown: 1`) |

## The arms

| arm (config) | settings file | what differs from `euroc_p_base` |
|---|---|---|
| `euroc_p_base` | `EuRoC_T1000.yaml` | plain stereo, 1000 features, `System.syncShutdown 1`, viewer off |
| `euroc_p_base_b` | `EuRoC_T1000.yaml` | nothing: the A/A replicate (same file, other name). Its comparison with `euroc_p_base` is the empirical false-positive rate |
| `euroc_p_c1` | `EuRoC_T1000_C1.yaml` | `GlobalBA.final 1` |
| `euroc_p_faithful` | `EuRoC_Dense_T1000.yaml` | `Dense.*` only: the thesis-faithful dense thread (unbounded queue, normal priority, `Dense.voxelSafe 0`) |
| `euroc_p_imp` | `EuRoC_Dense_Imp_T1000.yaml` | `Dense.*` (`queueLimit 3`, `lowPriority 1`, `voxelSafe 1`) and `GlobalBA.final 1` |
| `kitti_s_nosync` / `kitti_s_sync` | `KITTI04-12.yaml` / `KITTI04-12_Sync.yaml` | `System.syncShutdown` (the shutdown check on KITTI 04-10; 09 is the sequence whose last loop revisit is 2.5 s before the end) |

`euroc_p_c1` against `euroc_p_base` is C1 alone; `euroc_p_imp` against `euroc_p_c1` is the dense
thread plus its mitigations with the final BA held fixed (`--pair euroc_p_c1:euroc_p_imp`);
`euroc_p_imp` against `euroc_p_faithful` is A1/A2 and the voxel filter (`--pair euroc_p_faithful:euroc_p_imp`).
`python3 evaluation/harness/check_yaml_pairs.py` (host, needs only PyYAML) lists the keys that differ in each
pair, fails on any key outside the intended set, and checks that each arm holds its intended values
(faithful: every opt-in switch at its default). Run it after any edit of a settings file.

## The interleaving scheme

A *block* is one (run, sequence): every arm runs once in it. Execution order: for run `r`, for each
sequence, one design row = the order of the arms. Rows come from a **Williams design** (a balanced Latin
square): each arm occupies each position equally often, and each arm is immediately preceded by every
other arm equally often (even number of arms: n rows; odd: 2n rows = n cyclic shifts of a zigzag
sequence plus their reversals). The arm labels are shuffled by `--seed` (on the sorted names, so the
order given on the command line does not matter) and, per sequence, the rows are dealt to the runs in
cycles, the Latin blocks and the rows in them in random order.

* After every `n` runs (n = number of arms) each arm has had every position equally often *within each
  sequence*; after a whole cycle (n runs for even n, 2n for odd n; 5 arms: 10 runs) the carry-over is balanced too
  (inside a block: the arm that closes one block and the arm that opens the next are whatever the random rows give).
  `order.json` stores the position table and carry-over matrix; the run prints the position table and says
  when the counts are only balanced to within one (a run count that is not a multiple of n).
* Deterministic for a seed; raising `--runs` later (with `--resume`) keeps the runs already planned.
  Fix the seed before the confirmatory runs. `--design rotation` gives a plain cyclic Latin square.
* The point is that a drift over time (thermal state, a browser, the page cache) lands on all arms alike
  instead of on whichever block ran in the bad hour.

## Using it

```bash
# everything in one go (guards, preflight, pilot); nothing runs with --dry-run, --plan only validates
evaluation/run_pilot.sh --dry-run
evaluation/run_pilot.sh --experiment euroc --runs 10 --seed 1          # 5 arms x MH01,MH04,V103
evaluation/run_pilot.sh --experiment kitti --runs 10                   # kitti_s_nosync vs kitti_s_sync on 09,07
evaluation/run_pilot.sh --out DIR --resume --skip-preflight ...        # after an interruption (same arms, sequences, seed)
```

`run_pilot.sh` needs a benchmark copy (`ORB_WORK`, default `$HOME/orb_work_pilot`) built from this repo
(`evaluation/plan_33h.md` section 0). Per-machine settings are environment variables, listed in its header
(`ORB_CPUSET`, default `0,6,1,7` = SMT pairs (0,6) and (1,7) of the Ryzen 5 4600G: 2 physical cores, 4 threads,
the shape of the thesis' i7-7500U; `ORB_CPUSET=all` removes the limit). It refuses to start when less than 10 GB
are free, when another container of the image is running, or when another process uses more than 20 % of a
core, unless `--force`; the preflight cannot be forced past. The docker command has `--cap-add=SYS_NICE` **and**
`--ulimit nice=40:40`: with a non-root container user (`-u uid:gid`) the capability alone is not in the
process's effective set, and a thread that put itself in `SCHED_IDLE` then cannot go back (measured here:
`EPERM`); the ulimit is what makes the switch back work for an ordinary user.
The preflight also checks that the startup banner matches the arm (`preflight_expect` / `preflight_forbid` in
`sequences.yaml`): 1000 features everywhere; no dense banner in the baselines; for `euroc_p_imp` `queueLimit=3 lowPriority=1`
and `voxelSafe=1`; for `euroc_p_faithful` no `extensions` line at all (it is printed when any opt-in switch is on).

```bash
# the pieces, inside the image (numpy needed only for the runner)
python3 evaluation/harness/run_interleaved.py --arms A,B,C --sequences S1,S2 --runs N --seed K --out DIR \
    [--tag PREFIX] [--design williams|rotation] [--timeout S] [--prune-dense] [--resume] [--min-free-gb G]
python3 evaluation/harness/run_interleaved.py ... --dry-run          # inputs, schedule, estimate; runs nothing
python3 evaluation/harness/run_interleaved.py ... --preflight 25     # 'New Map created' required in every log
python3 evaluation/harness/run_interleaved.py --summary --out DIR    # audit view of order.json (no numpy)

# analysis (no numpy; deterministic)
python3 evaluation/harness/stats_compare.py --results DIR --out DIR/stats --baseline euroc_p_base \
    --arms euroc_p_base_b,euroc_p_c1,euroc_p_faithful,euroc_p_imp --null-arms euroc_p_base_b \
    --pair euroc_p_c1:euroc_p_imp --pair euroc_p_faithful:euroc_p_imp \
    --metrics ate_rmse,scores.kf_se3,scores.frames_sim3,log_metrics.tracking_time_mean_s,seconds
python3 evaluation/harness/run_benchmark.py --compare DIR/euroc_p_base DIR/euroc_p_c1     # unchanged
python3 evaluation/harness/log_metrics.py --table DIR/euroc_p_imp/MH04/run0*             # or JSON without --table
python3 evaluation/harness/log_metrics.py --self-test --logs /path/to/results             # parser vs raw counts
```

Exit status of `run_interleaved.py`: 0 all runs ok, 1 finished with failed runs (they are results, see
`--summary`), 3 stopped on low disk, 130 interrupted; 3 and 130 are resumable.

## What is written

`DIR/<arm>/results.json` is the format of `run_benchmark.py` (so `--compare` and the flat layout of
`stats_compare.py` read it unchanged; the original `stats_compare.py` also reads it when the directory is
named `<dataset>_<arm>`), rewritten after every run, plus:

* per run: `scores` = ATE RMSE of `frames_se3` (the primary `ate_rmse`), `frames_sim3`, `kf_se3`, `kf_sim3`
  (keyframe scores are `null` for KITTI, which writes no keyframe file), and `log_metrics` (see `log_metrics.py`
  docstring for every key; `null` = the log cannot tell, `0` = it can and the event did not occur);
* per run, when the arm's config in `sequences.yaml` has `run_expect` / `run_forbid` (regexes on the log of the
  finished run): `log_check` = `{ok, missing, forbidden}`. It is the proof that the binary honoured the settings,
  because a binary that ignores a key runs fine and prints nothing: `Shutdown: waited` (`System.syncShutdown`) in every
  `euroc_p_*` arm and in `kitti_s_sync` but not in `kitti_s_nosync`; `Final global bundle adjustment done` in
  `euroc_p_c1` and `euroc_p_imp` and in no other arm; the dense timing block in the dense arms. A failed check does not
  change a run's `status`, but the runner prints `LOG CHECK FAILED`, exits with 1, and `--summary` lists the runs;
* `meta`: `orb_cpuset`, `affinity_cpus` (what the process may really use), host load, CPU governor/boost/driver,
  sha256 of the binary and of the settings files, the command line, and the resume times.

`DIR/order.json`: the plan (arm, sequence, run, position) and, per run, `status`, `attempts`, start/end,
`returncode`, `seconds` and `host`: the CPU seconds that **other host processes** used on the allowed CPUs while
it ran (`other_busy_s` = `/proc/stat` busy time minus this container's cgroup usage), iowait, mean/min CPU frequency
and highest CPU temperature. In the smoke tests, on a machine that was compiling other jobs, it came out between 1 s
and 100 s per 30 s run; what a quiet machine shows has not been measured yet, so read the first pilot runs before
choosing a value. Treat `other_busy_s` as an indicator (kernel and accounting differences add a residue), not as a
measurement, and decide before the confirmatory runs which value excludes a run. A failed or interrupted attempt is kept as
`runNN.attemptK/`. The SLAM command is wrapped in `stdbuf -oL -eL`, so a crash or a kill does not lose the
buffered log (the 2046 s hung run of the first campaign left a 0-byte log).

## Not verified here, and limits

* The C++ side of `System.syncShutdown` and `Dense.voxelSafe` was still being written when these tools were made, so
  the parser was checked against logs of the work-in-progress copies, not against merged code: `Shutdown: waited 0.01 s
  for LocalMapping/LoopClosing/GBA`, `[System.syncShutdown] WARNING: gave up waiting after 3 s (...)`, `Global voxel
  filter (voxelSafe=1): 132 calls, 120 would overflow pcl's int32 grid, 120 handled by slabs, 273.30 s` (`, N left
  unfiltered` only when N > 0) and `Priority boosts refused: N`. It still accepts the earlier guess `Dense global voxel
  filter: ... N calls ... M overflows`, and keeps the raw line in `voxel_filter_summary`. If the merged C++ words a line
  differently, extend `log_metrics.py` and its `--self-test`; the `run_expect` patterns and the `voxelSafe=1` preflight
  pattern in `sequences.yaml` are the other places that hold the text.
* The frozen binary ignores both keys, so a run with it cannot show their effect. With it the `euroc_p_imp` preflight
  fails on purpose (`voxelSafe=1` is missing) and every `euroc_p_*` run fails its `log_check`.
* The author's timing table (thesis Table X: ORB extraction, local BA, ... per component) comes from a `REGISTER_TIMES`
  build, which this repo does not enable (it is a compile-time flag, not a settings key). The logs carry the dense stage
  times and, for TUM and KITTI, the mean/median tracking time; `stereo_euroc` fills `vTimesTrack` but prints no
  statistic, so on EuRoC `log_metrics.tracking_time_*` is `null` in every arm and the only timing is the wall time
  (`seconds`), which real-time playback dominates. The tracking time printed after a dense run has two decimals (the
  `std::fixed` stream state leaks; `tracking_time_low_precision`), so compare it across arms only when none is flagged.
