#!/bin/bash
# Interleaved, CPU-limited pilot: guards, preflight, then run_interleaved.py in the orb_slam3:dense image.
#
#   evaluation/run_pilot.sh [--experiment euroc|kitti|tum|kitti_confirm] [--dry-run] [--plan] [--force] [options]
#
# What a real run does, in this order:
#   0. refuses to start if the disk of ORB_WORK has less than ORB_MIN_FREE_GB (10) free, if a container of
#      ORB_IMAGE is already running, or if some other process uses more than ORB_HEAVY_PCT (20) % of one core
#      over a 3 s sample (the benchmark must own the machine) -- unless --force
#   1. preflight: every arm x sequence for ORB_PREFLIGHT_SECS (25) s; 'New Map created' has to be in each log.
#      A timeout alone proves nothing: a missing TUM association file once hung silently.
#   2. the pilot: all arms interleaved run by run (run_interleaved.py), each run scored at once.
# Everything is logged to $ORB_WORK/logs/pilot_<experiment>_<stamp>.log. systemd-inhibit keeps the machine
# awake. --dry-run prints the docker commands and runs nothing (no docker, no sampling, no files written).
#
# Experiments (arms and sequences can be overridden with --arms / --sequences):
#   euroc  euroc_p_base, euroc_p_base_b (A/A replicate), euroc_p_c1, euroc_p_faithful, euroc_p_imp on MH01,MH04,V103
#   kitti  kitti_s_nosync, kitti_s_sync (shutdown-synchronisation check) on 09,07
#   tum    tum_p_base, tum_p_c1, tum_p_faithful, tum_p_imp on fr1_desk,fr1_room,fr2_desk,fr2_large_no_loop,fr3_office
#          (thesis Table IV; evaluation/PRE_REGISTRATION_tum_kitti.md: --runs 12 --seed 13)
#   kitti_confirm  kitti_p_base, kitti_p_c1, kitti_p_faithful, kitti_p_imp on KITTI 00-10 (thesis Table VI;
#          the same pre-registration: --runs 4 --seed 17)
#
# Options (environment variable in brackets):
#   --runs N          runs per arm and sequence [ORB_RUNS, 10]; a multiple of the number of design rows
#                     (5 arms: 10; 2 arms: 2) keeps the position balance exact
#   --seed K          seed of the interleaving schedule [ORB_SEED, 1]; fix it before the confirmatory runs
#   --out DIR         result directory, inside ORB_WORK [ORB_WORK/results_pilot/<experiment>_<stamp>]
#   --tag PREFIX      prefix of the 'tag' written in each results.json
#   --resume          continue DIR (--out) after an interruption; arms, sequences, seed must be the same
#   --skip-preflight  do not run step 1;   --preflight-only  stop after step 1
#   --plan            validate every input and print the schedule inside the container (no SLAM), then stop
#   --force           ignore the disk and busy-machine guards of step 0 (never the preflight)
# Environment:
#   ORB_WORK          benchmark copy, mounted at /workspaces/ORB_SLAM3: it needs the build, the Vocabulary and the
#                     evaluation/harness of this repo [$HOME/orb_work_pilot]. Create it like plan_33h.md section 0.
#   ORB_CPUSET        docker --cpuset-cpus [0,6,1,7]: two physical cores / four threads of the Ryzen 5 4600G, whose
#                     SMT pairs are (0,6) (1,7) ... -- the same shape as the thesis' i7-7500U. "all" = no limit.
#   ORB_IMAGE [orb_slam3:dense]   ORB_DATASETS [$HOME/Datasets, mounted read-only]
#   ORB_RUN_TIMEOUT [1800]        per-run timeout in seconds
#   ORB_ULIMIT_NICE [40:40]       docker --ulimit nice=...: see the note at the docker flags; empty = off
#   ORB_MIN_FREE_GB [10]  ORB_HEAVY_PCT [20]  ORB_PREFLIGHT_SECS [25]  ORB_LOG_DIR [ORB_WORK/logs]
#
# Exit status: 0 done; 75 refused by a guard (nothing was run: safe to retry later); 2 usage error or failed
# preflight / plan; otherwise the status of run_interleaved.py (1 = finished with failed runs, 3 = stopped on low
# disk, 130 = interrupted; resume with --resume). Guard refusal used to be 1, the same code as "finished with
# failed runs": a retry loop must only retry 75.

set -u -o pipefail

EXPERIMENT=euroc
DRY=0; FORCE=0; PLAN=0; RESUME=0; SKIP_PF=0; PF_ONLY=0
ARMS=""; SEQS=""; OUT=""; TAG=""
RUNS=${ORB_RUNS:-10}
SEED=${ORB_SEED:-1}
ORB_WORK=${ORB_WORK:-$HOME/orb_work_pilot}
ORB_CPUSET=${ORB_CPUSET-0,6,1,7}
ORB_IMAGE=${ORB_IMAGE:-orb_slam3:dense}
ORB_DATASETS=${ORB_DATASETS:-$HOME/Datasets}
ORB_RUN_TIMEOUT=${ORB_RUN_TIMEOUT:-1800}
ORB_ULIMIT_NICE=${ORB_ULIMIT_NICE-40:40}
ORB_MIN_FREE_GB=${ORB_MIN_FREE_GB:-10}
ORB_HEAVY_PCT=${ORB_HEAVY_PCT:-20}
ORB_PREFLIGHT_SECS=${ORB_PREFLIGHT_SECS:-25}

die() { echo "run_pilot: $*" >&2; exit "${2:-2}"; }
usage() { sed -n '2,/^set -u/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'; }

while [ $# -gt 0 ]; do
  case "$1" in
    --experiment) EXPERIMENT=${2:?}; shift 2 ;;
    --arms) ARMS=${2:?}; shift 2 ;;
    --sequences) SEQS=${2:?}; shift 2 ;;
    --runs) RUNS=${2:?}; shift 2 ;;
    --seed) SEED=${2:?}; shift 2 ;;
    --out) OUT=${2:?}; shift 2 ;;
    --tag) TAG=${2:?}; shift 2 ;;
    --resume) RESUME=1; shift ;;
    --skip-preflight) SKIP_PF=1; shift ;;
    --preflight-only) PF_ONLY=1; shift ;;
    --plan) PLAN=1; shift ;;
    --force) FORCE=1; shift ;;
    --dry-run) DRY=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option $1 (see --help)" ;;
  esac
done

case "$EXPERIMENT" in
  euroc) : "${ARMS:=euroc_p_base,euroc_p_base_b,euroc_p_c1,euroc_p_faithful,euroc_p_imp}"; : "${SEQS:=MH01,MH04,V103}"
         BASELINE=euroc_p_base; NULL_ARMS=euroc_p_base_b; PAIRS=euroc_p_c1:euroc_p_imp,euroc_p_faithful:euroc_p_imp ;;
  kitti) : "${ARMS:=kitti_s_nosync,kitti_s_sync}"; : "${SEQS:=09,07}"
         BASELINE=kitti_s_nosync; NULL_ARMS=""; PAIRS="" ;;
  tum)   : "${ARMS:=tum_p_base,tum_p_c1,tum_p_faithful,tum_p_imp}"; : "${SEQS:=fr1_desk,fr1_room,fr2_desk,fr2_large_no_loop,fr3_office}"
         BASELINE=tum_p_base; NULL_ARMS=""; PAIRS=tum_p_c1:tum_p_imp,tum_p_faithful:tum_p_imp ;;
  kitti_confirm) : "${ARMS:=kitti_p_base,kitti_p_c1,kitti_p_faithful,kitti_p_imp}"; : "${SEQS:=00,01,02,03,04,05,06,07,08,09,10}"
         BASELINE=kitti_p_base; NULL_ARMS=""; PAIRS=kitti_p_c1:kitti_p_imp,kitti_p_faithful:kitti_p_imp ;;
  *) die "--experiment must be euroc, kitti, tum or kitti_confirm" ;;
esac
[ "$RUNS" -ge 1 ] 2>/dev/null || die "--runs must be a positive integer"

STAMP=$(date +%Y%m%d_%H%M%S)
if [ -z "$OUT" ]; then OUT=$ORB_WORK/results_pilot/${EXPERIMENT}_$STAMP; fi
case "$OUT" in /*) ;; *) OUT=$PWD/$OUT ;; esac
case "$OUT" in "$ORB_WORK"/*) OUT_REL=${OUT#"$ORB_WORK"/} ;; *) die "--out must be inside ORB_WORK ($ORB_WORK): the container only sees that" ;; esac
LOG_DIR=${ORB_LOG_DIR:-$ORB_WORK/logs}
LOG=$LOG_DIR/pilot_${EXPERIMENT}_$STAMP.log

# ---------------------------------------------------------------------------------------------------------
# docker command. --cap-add=SYS_NICE is requested so that the dense thread (Dense.lowPriority: SCHED_IDLE) can
# restore its scheduling class. A non-root container user (-u uid:gid, as the earlier drivers) does NOT get an
# added capability in its effective set -- tested on this machine: with --cap-add alone the switch back from
# SCHED_IDLE fails with EPERM -- so --ulimit nice=40:40 is passed too; that does make it work for a plain user.
# ---------------------------------------------------------------------------------------------------------
if [ -z "$ORB_CPUSET" ] || [ "$ORB_CPUSET" = all ] || [ "$ORB_CPUSET" = none ]; then CPUSET_LABEL=all; CPUSET_ARGS=();
else CPUSET_LABEL=$ORB_CPUSET; CPUSET_ARGS=(--cpuset-cpus "$ORB_CPUSET"); fi
docker_cmd() {  # docker_cmd NAME  -> the docker run prefix, one word per line in $DOCKER
  DOCKER=(docker run --rm --init --name "orb_pilot_${STAMP}_$1"
          -u "$(id -u):$(id -g)" -e HOME=/home/vscode
          -e LD_LIBRARY_PATH=/workspaces/ORB_SLAM3/lib:/workspaces/ORB_SLAM3/Thirdparty/DBoW2/lib:/workspaces/ORB_SLAM3/Thirdparty/g2o/lib
          --cap-add=SYS_NICE)
  [ -n "$ORB_ULIMIT_NICE" ] && DOCKER+=(--ulimit "nice=$ORB_ULIMIT_NICE")
  DOCKER+=("${CPUSET_ARGS[@]}" -e "ORB_CPUSET=$CPUSET_LABEL"
           -v "$ORB_WORK:/workspaces/ORB_SLAM3" -v "$ORB_DATASETS:/home/vscode/Datasets:ro"
           -w /workspaces/ORB_SLAM3 "$ORB_IMAGE")
}
HARNESS=(python3 evaluation/harness/run_interleaved.py --arms "$ARMS" --sequences "$SEQS" --seed "$SEED" --out "$OUT_REL")
[ -n "$TAG" ] && HARNESS+=(--tag "$TAG")

INHIBIT=()
if command -v systemd-inhibit >/dev/null 2>&1; then
  INHIBIT=(systemd-inhibit --what=sleep:idle:handle-lid-switch --who=orb_pilot --why="ORB-SLAM3 interleaved pilot" --mode=block)
fi
show() {  # print a command so that it can be pasted: quote only the words that need it
  local w
  for w in "$@"; do
    if [[ $w =~ ^[A-Za-z0-9_./:=,@%+-]+$ ]]; then printf '%s ' "$w"; else printf '%q ' "$w"; fi
  done
  echo
}

# ---------------------------------------------------------------------------------------------------------
# guards
# ---------------------------------------------------------------------------------------------------------
free_gb() {  # GiB free on the filesystem of $1 (or of its nearest existing parent)
  local d=$1
  while [ ! -e "$d" ] && [ "$d" != / ]; do d=$(dirname "$d"); done
  df -P -BG "$d" 2>/dev/null | awk 'NR==2 {gsub("G", "", $4); print $4}'
}

heavy_processes() {  # prints processes above ORB_HEAVY_PCT % of one core over 3 s; status 1 if there are any
  python3 - "$ORB_HEAVY_PCT" <<'PY'
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

check_guards() {  # returns the number of guards that said no
  local bad=0 free running
  free=$(free_gb "$ORB_WORK")
  if [ -z "$free" ]; then echo "guard: cannot read the free disk space of $ORB_WORK"; bad=$((bad + 1))
  elif [ "$free" -lt "$ORB_MIN_FREE_GB" ]; then echo "guard: only $free GB free under $ORB_WORK (< ORB_MIN_FREE_GB=$ORB_MIN_FREE_GB)"; bad=$((bad + 1))
  else echo "guard ok: $free GB free under $ORB_WORK (>= $ORB_MIN_FREE_GB)"; fi
  running=$(docker ps -q --filter "ancestor=$ORB_IMAGE" 2>/dev/null | wc -l)
  if [ "$running" -gt 0 ]; then echo "guard: $running container(s) of $ORB_IMAGE already running (docker ps)"; bad=$((bad + 1))
  else echo "guard ok: no container of $ORB_IMAGE is running"; fi
  echo "guard: sampling CPU use of all processes for 3 s (threshold ${ORB_HEAVY_PCT} % of one core) ..."
  if ! heavy_processes; then echo "guard: the processes above are using the CPU; close them or use --force"; bad=$((bad + 1))
  else echo "guard ok: no other process above ${ORB_HEAVY_PCT} % of a core"; fi
  return $bad
}

host_state() {
  echo "host: $(uname -srm); $(lscpu 2>/dev/null | sed -n 's/^Model name: *//p'); loadavg $(cut -d' ' -f1-3 /proc/loadavg)"
  echo "cpu policy: governor $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo ?), boost $(cat /sys/devices/system/cpu/cpufreq/boost 2>/dev/null || echo ?), driver $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_driver 2>/dev/null || echo ?)"
  echo "docker: $(docker --version 2>/dev/null); image $ORB_IMAGE $(docker image inspect -f '{{.Id}}' "$ORB_IMAGE" 2>/dev/null | cut -c1-19)"
}

# ---------------------------------------------------------------------------------------------------------
# dry run: print, run nothing
# ---------------------------------------------------------------------------------------------------------
if [ "$DRY" = 1 ]; then
  echo "DRY RUN: nothing is executed (no docker, no sampling, no files written)."
  echo "experiment $EXPERIMENT; arms $ARMS; sequences $SEQS; runs $RUNS; seed $SEED"
  echo "ORB_WORK $ORB_WORK$([ -d "$ORB_WORK" ] || echo '  (does not exist yet)'); out $OUT; log $LOG"
  echo "CPU limit: ORB_CPUSET=$CPUSET_LABEL; image $ORB_IMAGE; datasets $ORB_DATASETS (read-only)"
  echo "guards of a real run: free disk >= $ORB_MIN_FREE_GB GB (now: $(free_gb "$ORB_WORK" || true) GB), no container of $ORB_IMAGE, no process above $ORB_HEAVY_PCT % of a core (unless --force)"
  [ -n "${INHIBIT[*]:-}" ] && echo "wrapped in: ${INHIBIT[*]}" || echo "systemd-inhibit not found: the machine is not kept awake"
  echo
  EXTRA=(); [ "$RESUME" = 1 ] && EXTRA+=(--resume)
  docker_cmd plan;      echo "# --plan (schedule and input check, no SLAM):";  show "${INHIBIT[@]}" "${DOCKER[@]}" "${HARNESS[@]}" --runs "$RUNS" --dry-run
  [ "$SKIP_PF" = 1 ] || { docker_cmd preflight; echo; echo "# step 1, preflight (${ORB_PREFLIGHT_SECS} s per arm x sequence):"; show "${INHIBIT[@]}" "${DOCKER[@]}" "${HARNESS[@]}" --timeout "$ORB_PREFLIGHT_SECS" --preflight "$ORB_PREFLIGHT_SECS"; }
  if [ "$PF_ONLY" != 1 ]; then docker_cmd pilot; echo; echo "# step 2, the pilot:"; show "${INHIBIT[@]}" "${DOCKER[@]}" "${HARNESS[@]}" --runs "$RUNS" --timeout "$ORB_RUN_TIMEOUT" --prune-dense "${EXTRA[@]}"; fi
  exit 0
fi

# ---------------------------------------------------------------------------------------------------------
# real run
# ---------------------------------------------------------------------------------------------------------
command -v docker >/dev/null 2>&1 || die "docker not found"
[ -d "$ORB_WORK" ] || die "ORB_WORK=$ORB_WORK does not exist: make the benchmark copy first (evaluation/plan_33h.md section 0)"
for f in Vocabulary/ORBvoc.txt evaluation/harness/run_interleaved.py evaluation/harness/sequences.yaml; do
  [ -e "$ORB_WORK/$f" ] || die "ORB_WORK has no $f (is it a benchmark copy built from this repo?)"
done
[ -d "$ORB_DATASETS" ] || die "ORB_DATASETS=$ORB_DATASETS does not exist"
docker image inspect "$ORB_IMAGE" >/dev/null 2>&1 || die "docker image $ORB_IMAGE not found"
case "$CPUSET_LABEL" in all) ;; *[!0-9,-]*) die "ORB_CPUSET=$ORB_CPUSET is not a CPU list like 0,6,1,7" ;; esac
mkdir -p "$LOG_DIR"
exec > >(tee -a "$LOG") 2>&1
echo "run_pilot $(date '+%F %T'): experiment $EXPERIMENT; arms $ARMS; sequences $SEQS; runs $RUNS; seed $SEED"
echo "ORB_WORK $ORB_WORK; out $OUT; log $LOG; CPU limit ORB_CPUSET=$CPUSET_LABEL"
host_state

if [ "$FORCE" = 1 ]; then echo "guards skipped (--force)"
else
  check_guards || { echo "run_pilot: refusing to start (see above). Free the machine, or pass --force to override."; exit 75; }
fi

run_docker() {  # run_docker NAME ARGS...  (inside the inhibitor)
  local name=$1; shift
  docker_cmd "$name"
  echo "+ $(show "${DOCKER[@]}" "$@")"
  "${INHIBIT[@]}" "${DOCKER[@]}" "$@"
}

if [ "$PLAN" = 1 ]; then
  run_docker plan "${HARNESS[@]}" --runs "$RUNS" --dry-run; rc=$?
  echo "plan status $rc"; exit $((rc == 0 ? 0 : 2))
fi

if [ "$SKIP_PF" != 1 ]; then
  echo; echo "== step 1: preflight, ${ORB_PREFLIGHT_SECS} s per arm x sequence =="
  run_docker preflight "${HARNESS[@]}" --timeout "$ORB_PREFLIGHT_SECS" --preflight "$ORB_PREFLIGHT_SECS"; rc=$?
  echo "preflight status $rc"
  [ $rc -eq 0 ] || { echo "run_pilot: preflight FAILED: fix it before any long run. Nothing else was run."; exit 2; }
  [ "$PF_ONLY" = 1 ] && exit 0
fi

echo; echo "== step 2: the pilot =="
EXTRA=(); [ "$RESUME" = 1 ] && EXTRA+=(--resume)
echo "$(date '+%F %T') START pilot $EXPERIMENT" >> "$LOG_DIR/progress.log"
run_docker pilot "${HARNESS[@]}" --runs "$RUNS" --timeout "$ORB_RUN_TIMEOUT" --prune-dense "${EXTRA[@]}"; rc=$?
echo "$(date '+%F %T') END   pilot $EXPERIMENT rc=$rc" >> "$LOG_DIR/progress.log"
echo; echo "pilot status $rc"
python3 "$ORB_WORK/evaluation/harness/run_interleaved.py" --summary --out "$OUT" || true
echo
echo "analysis (stats_compare.py needs no numpy and is deterministic; --jobs 4 speeds it up):"
in_arms() { case ",$ARMS," in *",$1,"*) return 0 ;; *) return 1 ;; esac; }
in_arms "$BASELINE" || BASELINE=${ARMS%%,*}
null_ok=(); for a in ${NULL_ARMS//,/ }; do in_arms "$a" && null_ok+=("$a"); done
pairs_ok=(); for pr in ${PAIRS//,/ }; do in_arms "${pr%%:*}" && in_arms "${pr##*:}" && pairs_ok+=("$pr"); done
join() { local IFS=,; echo "$*"; }
echo "  nice -n 19 python3 $ORB_WORK/evaluation/harness/stats_compare.py --results $OUT --out $OUT/stats --baseline $BASELINE \\"
echo "      --arms $(join $(printf '%s\n' ${ARMS//,/ } | grep -vx "$BASELINE")) ${null_ok[*]:+--null-arms $(join "${null_ok[@]}") }${pairs_ok[*]:+--pair $(join "${pairs_ok[@]}") }\\"
echo "      --metrics ate_rmse,t_rel_pct,scores.kf_se3,scores.frames_sim3,log_metrics.tracking_time_mean_s,seconds"
exit $rc
