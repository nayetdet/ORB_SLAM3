#!/bin/bash
# Applies the voxelSafe filter offline to a saved cloud (read-only input) and verifies the
# result independently with numpy. Run inside orb_slam3:dense after the library is built:
#
#   docker run --rm -u 1000:1000 -e HOME=/home/vscode \
#       -v <repo copy>:/workspaces/ORB_SLAM3 -v <scratch dir>:/scr \
#       -v <results dir>:/results:ro -w /workspaces/ORB_SLAM3 orb_slam3:dense \
#       bash evaluation/tests/run_voxel_safe_cloud.sh /scr /results/<run>/<cloud>.pcd <leaf> <tag> [mode [K]]
#
# mode: safe (default; one call + numpy check), legacy (one voxelSafe=0 call), replay
# (K dense-thread calls with voxelSafe=1, K = keyframes / 10), replay-legacy (the same with 0).
#
# Needs numpy and roughly 8 GB of RAM for a 67M-point cloud. The filtered cloud is written to
# <scratch>/<tag>.filtered.raw (about 16 bytes per output point) and deleted at the end.
set -e
ROOT=${ROOT:-/workspaces/ORB_SLAM3}
OUT=$1
PCD=$2
LEAF=$3
TAG=${4:-cloud}
MODE=${5:-safe}
mkdir -p "$OUT"

if [ ! -x "$OUT/voxel_safe_cloud" ] || [ "$ROOT/src/PointCloudMapping.cc" -nt "$OUT/voxel_safe_cloud" ] ||
   [ "$ROOT/evaluation/tests/voxel_safe_cloud.cc" -nt "$OUT/voxel_safe_cloud" ]; then
    FL=$ROOT/build/CMakeFiles/ORB_SLAM3.dir/flags.make
    DEFS=$(sed -n 's/^CXX_DEFINES = //p' "$FL")
    INCS=$(sed -n 's/^CXX_INCLUDES = //p' "$FL")
    FLAGS=$(sed -n 's/^CXX_FLAGS = //p' "$FL")
    CUSTOM=$(sed -n 's/^# Custom flags: .*PointCloudMapping.cc.o_FLAGS = //p' "$FL")
    LINK=$ROOT/build/CMakeFiles/stereo_kitti.dir/link.txt
    RPATH=$(grep -o -e '-Wl,-rpath,[^ ]*' "$LINK" | head -1)
    LIBS="$RPATH $(sed -E 's/.* -Wl,-rpath,[^ ]+ //' "$LINK")"
    echo "== compiling voxel_safe_cloud"
    eval nice -n 19 c++ $DEFS $CUSTOM -I"$ROOT/src" $INCS $FLAGS -DPCL_SILENCE_MALLOC_WARNING=1 \
        "$ROOT/evaluation/tests/voxel_safe_cloud.cc" -o "$OUT/voxel_safe_cloud" $LIBS > "$OUT/compile_cloud.log" 2>&1 ||
        { grep -n "error\|undefined" "$OUT/compile_cloud.log" | head; exit 1; }
fi

K=${6:-100}
case "$MODE" in
legacy)
    echo "== legacy single pass (what voxelSafe=0 does) on $PCD, leaf $LEAF"
    nice -n 19 "$OUT/voxel_safe_cloud" "$PCD" "$LEAF" --legacy
    exit 0 ;;
replay)
    echo "== replay of $K dense-thread filter calls, voxelSafe=1, on $PCD, leaf $LEAF"
    nice -n 19 "$OUT/voxel_safe_cloud" "$PCD" "$LEAF" --replay "$K"
    exit 0 ;;
replay-legacy)
    echo "== replay of $K dense-thread filter calls, voxelSafe=0, on $PCD, leaf $LEAF"
    nice -n 19 "$OUT/voxel_safe_cloud" "$PCD" "$LEAF" --legacy --replay "$K"
    exit 0 ;;
esac

echo "== voxelSafe filter on $PCD, leaf $LEAF"
nice -n 19 "$OUT/voxel_safe_cloud" "$PCD" "$LEAF" --out "$OUT/$TAG.filtered.raw"
echo "== independent numpy check"
nice -n 19 python3 "$ROOT/evaluation/tests/voxel_count_check.py" --pcd "$PCD" --leaf "$LEAF" \
    --filtered "$OUT/$TAG.filtered.raw" --sample 200000
rm -f "$OUT/$TAG.filtered.raw"
