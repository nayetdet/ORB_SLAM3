#!/bin/bash
# Two extra experiments, unattended: guard-retry, preflight, interleaved runs, stats, markers.
#
#   evaluation/run_extra.sh [--fps] [--multi] [--after MARKER] [--dry-run] [--plan] [--force] [--skip-preflight]
#
#   --fps    (i)  UNPACED throughput (Zhang 2023, Table IV / X "FPS"): ORB_NO_PACING=1, arms base / c1 / faithful / imp,
#                 one TUM (fr1_desk), one EuRoC (MH01), one KITTI (07) sequence; 3 blocks, one per dataset.
#                 Registry evaluation/harness/sequences_fps.yaml.  Marker FPS_DONE.
#   --multi  (ii) MULTI-SEQUENCE merging (Zhang 2023, sec. 3.5 / 4.3 / Table VII): the EuRoC chains MH01-02..MH01-05,
#                 V101-102, V101-103 and the TUM chain fr2_large_with_loop -> fr2_large_no_loop, each in ONE process
#                 (ORB-SLAM3 merges the maps itself), plus the single-sequence controls; same four arms.
#                 Registry evaluation/harness/sequences_multi.yaml.  Marker MULTI_DONE.
#   no flag  both, --fps first.
#   --after MARKER   wait (poll every 60 s) until MARKER is a line of the progress log before starting (e.g. CONFIRMATORY_DONE)
#   --dry-run        print every command, run nothing (no docker, no sampling, no files)
#   --plan           validate inputs and print the schedules inside the container (no SLAM), then stop
#   --force          skip the busy-machine guards;  --skip-preflight  skip the 25 s startup check of every arm x sequence
#
# Everything runs in docker $ORB_IMAGE on ORB_CPUSET (default 0,6,1,7: two physical cores, as the confirmatory run) from the
# benchmark copy ORB_WORK. Per block: guards (retried every ORB_GUARD_WAIT s, at most ORB_GUARD_RETRIES times) -> preflight ->
# run_interleaved.py (resumes by itself when its --out directory already has an order.json) -> stats_compare.py + extra_report.py.
# A block that finishes with failed / INVALID runs is NOT retried (run_interleaved returns 1 for that; here a refused guard is a
# separate, internal status, so the two cannot be confused as they can with run_pilot.sh). Results: $ORB_WORK/results_extra/
# <block>/, stats in <block>/stats, reports <block>/fps_report.md | chain_report.md. Progress: $ORB_WORK/logs/pilot_progress.log.
#
# Environment (defaults): ORB_WORK $HOME/orb_work2; ORB_CPUSET 0,6,1,7; ORB_IMAGE orb_slam3:dense; ORB_DATASETS $HOME/Datasets;
#   ORB_FPS_RUNS 12; ORB_FPS_RUNS_KITTI 4; ORB_MULTI_RUNS_EUROC 6; ORB_MULTI_RUNS_TUM 8; ORB_FPS_SEED 21; ORB_MULTI_SEED 22; ORB_RUN_TIMEOUT 1800 (multi
#   chains get x3); ORB_PREFLIGHT_SECS 25; ORB_MIN_FREE_GB 10; ORB_HEAVY_PCT 20; ORB_ULIMIT_NICE 40:40; ORB_GUARD_RETRIES 72;
#   ORB_GUARD_WAIT 300; ORB_PROGRESS_LOG $ORB_WORK/logs/pilot_progress.log; ORB_OUT_ROOT $ORB_WORK/results_extra.
#
# Exit status: 0 done; 1 some block finished with failed/invalid runs or aborted (see the log); 2 usage error.

set -u -o pipefail

DO_FPS=0; DO_MULTI=0; AFTER=""; DRY=0; PLAN=0; FORCE=0; SKIP_PF=0
while [ $# -gt 0 ]; do
  case "$1" in
    --fps) DO_FPS=1; shift ;;
    --multi) DO_MULTI=1; shift ;;
    --after) AFTER=${2:?}; shift 2 ;;
    --dry-run) DRY=1; shift ;;
    --plan) PLAN=1; shift ;;
    --force) FORCE=1; shift ;;
    --skip-preflight) SKIP_PF=1; shift ;;
    -h|--help) sed -n '2,/^set -u/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "run_extra: unknown option $1 (see --help)" >&2; exit 2 ;;
  esac
done
[ $DO_FPS = 0 ] && [ $DO_MULTI = 0 ] && { DO_FPS=1; DO_MULTI=1; }

ORB_WORK=${ORB_WORK:-$HOME/orb_work2}
ORB_CPUSET=${ORB_CPUSET-0,6,1,7}
ORB_IMAGE=${ORB_IMAGE:-orb_slam3:dense}
ORB_DATASETS=${ORB_DATASETS:-$HOME/Datasets}
FPS_RUNS=${ORB_FPS_RUNS:-12}; FPS_RUNS_KITTI=${ORB_FPS_RUNS_KITTI:-4}; MULTI_RUNS_EUROC=${ORB_MULTI_RUNS_EUROC:-6}; MULTI_RUNS_TUM=${ORB_MULTI_RUNS_TUM:-8}
FPS_SEED=${ORB_FPS_SEED:-21}; MULTI_SEED=${ORB_MULTI_SEED:-22}
RUN_TIMEOUT=${ORB_RUN_TIMEOUT:-1800}
PF_SECS=${ORB_PREFLIGHT_SECS:-25}
MIN_FREE_GB=${ORB_MIN_FREE_GB:-10}; HEAVY_PCT=${ORB_HEAVY_PCT:-20}
ULIMIT_NICE=${ORB_ULIMIT_NICE-40:40}
GUARD_RETRIES=${ORB_GUARD_RETRIES:-72}; GUARD_WAIT=${ORB_GUARD_WAIT:-300}
OUT_ROOT=${ORB_OUT_ROOT:-$ORB_WORK/results_extra}
PROGRESS=${ORB_PROGRESS_LOG:-$ORB_WORK/logs/pilot_progress.log}
LOG_DIR=$ORB_WORK/logs
STAMP=$(date +%Y%m%d_%H%M%S)

ARMS_SUFFIX=(base c1 faithful imp)
FPS_REG=evaluation/harness/sequences_fps.yaml
MULTI_REG=evaluation/harness/sequences_multi.yaml
MULTI_EUROC_SEQS=MH01-02,MH01-03,MH01-04,MH01-05,V101-102,V101-103,MH02,MH03,MH04,MH05,V102,V103
MULTI_TUM_SEQS=fr2_loop-noloop,fr2_noloop

arms_of() {  # arms_of PREFIX -> PREFIX_base,PREFIX_c1,...
  local a=() s; for s in "${ARMS_SUFFIX[@]}"; do a+=("$1_$s"); done; local IFS=,; echo "${a[*]}"
}

log() { echo "$(date '+%F %T') $*" >> "$PROGRESS"; echo "run_extra $(date '+%F %T') $*"; }

if [ "$ORB_CPUSET" = all ] || [ "$ORB_CPUSET" = none ] || [ -z "$ORB_CPUSET" ]; then CPU_ARGS=(); CPU_LABEL=all
else CPU_ARGS=(--cpuset-cpus "$ORB_CPUSET"); CPU_LABEL=$ORB_CPUSET; fi

docker_prefix() {  # docker_prefix NAME [extra docker args...]  -> array DOCKER
  local name=$1; shift
  DOCKER=(docker run --rm --init --name "orb_extra_${STAMP}_$name" -u "$(id -u):$(id -g)" -e HOME=/home/vscode
          -e LD_LIBRARY_PATH=/workspaces/ORB_SLAM3/lib:/workspaces/ORB_SLAM3/Thirdparty/DBoW2/lib:/workspaces/ORB_SLAM3/Thirdparty/g2o/lib
          --cap-add=SYS_NICE)
  [ -n "$ULIMIT_NICE" ] && DOCKER+=(--ulimit "nice=$ULIMIT_NICE")
  DOCKER+=("${CPU_ARGS[@]}" -e "ORB_CPUSET=$CPU_LABEL" "$@"
           -v "$ORB_WORK:/workspaces/ORB_SLAM3" -v "$ORB_DATASETS:/home/vscode/Datasets:ro" -w /workspaces/ORB_SLAM3 "$ORB_IMAGE")
}
INHIBIT=()
command -v systemd-inhibit >/dev/null 2>&1 && INHIBIT=(systemd-inhibit --what=sleep:idle:handle-lid-switch --who=orb_extra --why="ORB-SLAM3 extra experiments" --mode=block)

show() { local w; for w in "$@"; do if [[ $w =~ ^[A-Za-z0-9_./:=,@%+-]+$ ]]; then printf '%s ' "$w"; else printf '%q ' "$w"; fi; done; echo; }

# ---------------------------------------------------------------------------------------------------------------------
# guards (the same three as run_pilot.sh): free disk, no other container of the image, no busy process. 0 = ok, 1 = refused.
# ---------------------------------------------------------------------------------------------------------------------
free_gb() { local d=$1; while [ ! -e "$d" ] && [ "$d" != / ]; do d=$(dirname "$d"); done; df -P -BG "$d" 2>/dev/null | awk 'NR==2 {gsub("G","",$4); print $4}'; }
heavy_processes() {
  python3 - "$HEAVY_PCT" <<'PY'
import os, sys, time
thr, hz, dur = float(sys.argv[1]), os.sysconf("SC_CLK_TCK"), 3.0
me = {os.getpid(), os.getppid()}
def snap():
    out = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) in me:
            continue
        try:
            with open("/proc/%s/stat" % pid) as fh:
                s = fh.read()
            i = s.rindex(")")
            f = s[i + 2:].split()
            out[int(pid)] = (s[s.index("(") + 1:i], int(f[11]) + int(f[12]))
        except (OSError, ValueError, IndexError):
            pass
    return out
a, t0 = snap(), time.time()
time.sleep(dur)
b, dt = snap(), time.time() - t0
rows = sorted(((100.0 * (b[p][1] - a[p][1]) / hz / dt, p, b[p][0]) for p in b if p in a), reverse=True)
heavy = [r for r in rows if r[0] > thr]
for pct, pid, comm in heavy[:8]:
    print("  %5.1f %%  pid %-8d %s" % (pct, pid, comm))
sys.exit(1 if heavy else 0)
PY
}
guard_ok() {
  local bad=0 free running
  free=$(free_gb "$ORB_WORK")
  if [ -z "$free" ]; then echo "guard: cannot read the free disk space of $ORB_WORK"; bad=1
  elif [ "$free" -lt "$MIN_FREE_GB" ]; then echo "guard: only $free GB free under $ORB_WORK (< ORB_MIN_FREE_GB=$MIN_FREE_GB)"; bad=1; fi
  running=$(docker ps -q --filter "ancestor=$ORB_IMAGE" 2>/dev/null | wc -l)
  if [ "$running" -gt 0 ]; then echo "guard: $running container(s) of $ORB_IMAGE already running"; bad=1; fi
  if [ "$FORCE" = 1 ]; then return 0; fi
  heavy_processes || { echo "guard: the processes above use the CPU"; bad=1; }
  return $bad
}
guard_retry() {  # guard_retry LABEL ; 0 = clear to go, 1 = gave up
  local n=0
  if [ "$FORCE" = 1 ]; then return 0; fi
  while ! guard_ok; do
    n=$((n + 1))
    [ $n -gt "$GUARD_RETRIES" ] && { log "$1: guard still refusing after $GUARD_RETRIES retries, giving up"; return 1; }
    log "$1: guard refused, retry $n in $((GUARD_WAIT / 60)) min"
    sleep "$GUARD_WAIT"
  done
  return 0
}

# ---------------------------------------------------------------------------------------------------------------------
# one block = one run_interleaved.py invocation (one dataset: stats_compare's flat layout compares arms of one dataset)
#   block NAME REGISTRY ARMS SEQS RUNS SEED KIND(fps|multi) TIMEOUT [extra docker -e args...]
# ---------------------------------------------------------------------------------------------------------------------
BLOCK_FAILS=0
block() {
  local name=$1 reg=$2 arms=$3 seqs=$4 runs=$5 seed=$6 kind=$7 tmo=$8; shift 8
  local out=$OUT_ROOT/$name out_rel=${OUT_ROOT#"$ORB_WORK"/}/$name
  local H=(python3 evaluation/harness/run_interleaved.py --registry "$reg" --arms "$arms" --sequences "$seqs" --seed "$seed" --out "$out_rel" --tag "extra_$name")
  if [ "$DRY" = 1 ]; then
    echo; echo "# block $name: $kind, arms $arms, sequences $seqs, runs $runs, seed $seed, out $out"
    docker_prefix plan "$@"; echo "# schedule + input check (inside the image, no SLAM):"; show "${DOCKER[@]}" "${H[@]}" --runs "$runs" --dry-run
    [ $SKIP_PF = 1 ] || { docker_prefix pf "$@"; echo "# preflight:"; show "${INHIBIT[@]}" "${DOCKER[@]}" "${H[@]}" --timeout "$PF_SECS" --preflight "$PF_SECS"; }
    docker_prefix run "$@"; echo "# the run:"; show "${INHIBIT[@]}" "${DOCKER[@]}" "${H[@]}" --runs "$runs" --timeout "$tmo" --prune-dense
    return 0
  fi
  mkdir -p "$OUT_ROOT" "$LOG_DIR"
  local blog=$LOG_DIR/extra_${name}_$STAMP.log
  log "block $name START ($kind; arms $arms; sequences $seqs; runs $runs; seed $seed; cpuset $CPU_LABEL); log $blog"
  {
    guard_retry "$name" || { log "block $name ABORTED (guards)"; BLOCK_FAILS=$((BLOCK_FAILS + 1)); return 0; }
    docker_prefix plan "$@"
    "${DOCKER[@]}" "${H[@]}" --runs "$runs" --dry-run > "$LOG_DIR/extra_${name}_plan.txt" 2>&1 || {
      log "block $name ABORTED: --dry-run found missing inputs, see $LOG_DIR/extra_${name}_plan.txt"; BLOCK_FAILS=$((BLOCK_FAILS + 1)); return 0; }
    if [ "$PLAN" = 1 ]; then cat "$LOG_DIR/extra_${name}_plan.txt"; return 0; fi
    local resume=(); [ -f "$out/order.json" ] && resume=(--resume) && log "block $name: $out/order.json exists, resuming"
    if [ $SKIP_PF != 1 ] && [ ${#resume[@]} = 0 ]; then
      docker_prefix pf "$@"
      "${INHIBIT[@]}" "${DOCKER[@]}" "${H[@]}" --timeout "$PF_SECS" --preflight "$PF_SECS"
      local prc=$?
      [ $prc -eq 0 ] || { log "block $name ABORTED: preflight failed (rc=$prc)"; BLOCK_FAILS=$((BLOCK_FAILS + 1)); return 0; }
    fi
    docker_prefix run "$@"
    echo "+ $(show "${DOCKER[@]}" "${H[@]}" --runs "$runs" --timeout "$tmo" --prune-dense "${resume[@]}")"
    "${INHIBIT[@]}" "${DOCKER[@]}" "${H[@]}" --runs "$runs" --timeout "$tmo" --prune-dense "${resume[@]}"
    local rc=$?
    log "block $name run END rc=$rc"   # 0 done; 1 finished with failed / INVALID runs (kept, never retried); 3 low disk; 130 interrupted
    [ $rc -eq 0 ] || BLOCK_FAILS=$((BLOCK_FAILS + 1))
    python3 evaluation/harness/run_interleaved.py --summary --out "$out" || true
    analyse "$name" "$kind" "$arms" "$out"
  } >> "$blog" 2>&1
  tail -n 5 "$blog" | sed 's/^/    | /'
}

analyse() {  # analyse NAME KIND ARMS OUT : stats_compare.py (no numpy) + extra_report.py, on the host
  local name=$1 kind=$2 arms=$3 out=$4 first=${3%%,*}
  local rest=${arms#*,} metrics
  local pre=${first%_base}
  local pairs="${pre}_c1:${pre}_imp,${pre}_faithful:${pre}_imp"
  if [ "$kind" = fps ]; then
    metrics=log_metrics.frame_time_ms,ate_rmse; [ "$pre" = kitti_f ] && metrics=log_metrics.frame_time_ms,t_rel_pct
  else
    metrics=ate_rmse,scores.last_mean_m,scores.chain_se3,scores.last_in_chain_se3
  fi
  [ -d "$out" ] || return 0
  nice -n 19 python3 "$ORB_WORK/evaluation/harness/stats_compare.py" --results "$out" --layout flat --baseline "$first" \
      --arms "$rest" --pair "$pairs" --metrics "$metrics" --jobs 2 --out "$out/stats" > "$LOG_DIR/stats_extra_$name.out" 2>&1
  echo "stats_compare rc=$? -> $out/stats"
  python3 "$ORB_WORK/evaluation/harness/extra_report.py" "$kind" --results "$out" > "$out/${kind/multi/chain}_report.md" 2>"$LOG_DIR/report_extra_$name.err"
  echo "extra_report rc=$? -> $out/${kind/multi/chain}_report.md"
}

# ---------------------------------------------------------------------------------------------------------------------
if [ "$DRY" != 1 ]; then
  command -v docker >/dev/null 2>&1 || { echo "run_extra: docker not found" >&2; exit 2; }
  [ -d "$ORB_WORK" ] || { echo "run_extra: ORB_WORK=$ORB_WORK does not exist" >&2; exit 2; }
  for f in Vocabulary/ORBvoc.txt evaluation/harness/run_interleaved.py evaluation/harness/extra_report.py $FPS_REG $MULTI_REG Examples/RGB-D/TUM1_Sync.yaml Examples/RGB-D/TUM2_Dense_Multi_Sync.yaml; do
    [ -e "$ORB_WORK/$f" ] || { echo "run_extra: ORB_WORK has no $f (is it a copy of the branch with this commit, rebuilt?)" >&2; exit 2; }
  done
  docker image inspect "$ORB_IMAGE" >/dev/null 2>&1 || { echo "run_extra: docker image $ORB_IMAGE not found" >&2; exit 2; }
  mkdir -p "$LOG_DIR"
  cd "$ORB_WORK" || exit 2
  if [ -n "$AFTER" ]; then
    log "waiting for $AFTER in $PROGRESS"
    until grep -q "$AFTER" "$PROGRESS" 2>/dev/null; do sleep 60; done
  fi
  for b in Examples/Stereo/stereo_euroc Examples/Stereo/stereo_kitti Examples/RGB-D/rgbd_tum; do
    grep -qa ORB_NO_PACING "$ORB_WORK/$b" 2>/dev/null || {
      echo "run_extra: $ORB_WORK/$b does not know ORB_NO_PACING: rebuild ORB_WORK from this commit first" >&2; exit 2; }
  done
fi

if [ $DO_FPS = 1 ]; then
  [ "$DRY" = 1 ] || log "FPS experiment START (unpaced, ORB_NO_PACING=1)"
  block fps_tum   $FPS_REG "$(arms_of tum_f)"   fr1_desk "$FPS_RUNS"       "$FPS_SEED" fps "$RUN_TIMEOUT" -e ORB_NO_PACING=1
  block fps_euroc $FPS_REG "$(arms_of euroc_f)" MH01     "$FPS_RUNS"       "$FPS_SEED" fps "$RUN_TIMEOUT" -e ORB_NO_PACING=1
  block fps_kitti $FPS_REG "$(arms_of kitti_f)" 07       "$FPS_RUNS_KITTI" "$FPS_SEED" fps "$RUN_TIMEOUT" -e ORB_NO_PACING=1
  [ "$DRY" = 1 ] || { [ "$PLAN" = 1 ] || log "FPS_DONE"; }
fi
if [ $DO_MULTI = 1 ]; then
  [ "$DRY" = 1 ] || log "MULTI experiment START (chains in one process, paced)"
  block multi_tum   $MULTI_REG "$(arms_of tum_m)"   "$MULTI_TUM_SEQS"   "$MULTI_RUNS_TUM" "$MULTI_SEED" multi $((RUN_TIMEOUT * 3))
  block multi_euroc $MULTI_REG "$(arms_of euroc_m)" "$MULTI_EUROC_SEQS" "$MULTI_RUNS_EUROC" "$MULTI_SEED" multi $((RUN_TIMEOUT * 3))
  [ "$DRY" = 1 ] || { [ "$PLAN" = 1 ] || log "MULTI_DONE"; }
fi
[ "$DRY" = 1 ] && echo "# dry run: nothing executed"
[ "$BLOCK_FAILS" -eq 0 ] && exit 0 || exit 1
