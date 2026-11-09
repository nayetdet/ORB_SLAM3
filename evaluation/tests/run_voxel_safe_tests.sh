#!/bin/bash
# Builds and runs the Dense.voxelSafe tests inside orb_slam3:dense.
#
#   docker run --rm -u 1000:1000 -e HOME=/home/vscode \
#       -v <repo copy>:/workspaces/ORB_SLAM3 -v <scratch dir>:/scr \
#       -w /workspaces/ORB_SLAM3 orb_slam3:dense \
#       bash evaluation/tests/run_voxel_safe_tests.sh /scr
#
# Needs the library built first in <repo copy>/build with -DDENSE_RECONSTRUCTION=ON (the
# recipe in evaluation/plan_33h.md, section 0): the tests take their compile flags from
# CMake's flags.make and link against lib/libORB_SLAM3.so.
#
# Steps: compile voxel_safe_test.cc (it #includes src/PointCloudMapping.cc), run it, then
# count the voxels of its synthetic cloud independently with numpy.
set -e
ROOT=${ROOT:-/workspaces/ORB_SLAM3}
OUT=${1:-/tmp/voxel_safe_tests}
mkdir -p "$OUT"

FL=$ROOT/build/CMakeFiles/ORB_SLAM3.dir/flags.make
DEFS=$(sed -n 's/^CXX_DEFINES = //p' "$FL")
INCS=$(sed -n 's/^CXX_INCLUDES = //p' "$FL")
FLAGS=$(sed -n 's/^CXX_FLAGS = //p' "$FL")
CUSTOM=$(sed -n 's/^# Custom flags: .*PointCloudMapping.cc.o_FLAGS = //p' "$FL")
# The libraries an example links with (the .cc pulls in KeyFrame/System, hence pangolin etc.).
LINK=$ROOT/build/CMakeFiles/stereo_kitti.dir/link.txt
RPATH=$(grep -o -e '-Wl,-rpath,[^ ]*' "$LINK" | head -1)
LIBS="$RPATH $(sed -E 's/.* -Wl,-rpath,[^ ]+ //' "$LINK")"

# EXTRA_FLAGS, e.g. "-fsanitize=address,undefined -fno-omit-frame-pointer", is appended to the compile line;
# BIN names the executable so a sanitised build does not overwrite the plain one.
BIN=${BIN:-voxel_safe_test}
echo "== compiling $BIN (flags of the library: $FLAGS $CUSTOM $EXTRA_FLAGS)"
if ! eval nice -n 19 c++ $DEFS $CUSTOM -I"$ROOT/src" $INCS $FLAGS $EXTRA_FLAGS -DPCL_SILENCE_MALLOC_WARNING=1 \
        "$ROOT/evaluation/tests/voxel_safe_test.cc" -o "$OUT/$BIN" $LIBS $EXTRA_FLAGS > "$OUT/compile.log" 2>&1
then
    grep -n "error\|undefined" "$OUT/compile.log" | head -30
    exit 1
fi
# warnings of the test and of the file under test; third-party header noise stays in compile.log
grep -n "evaluation/tests/.*warning\|src/PointCloudMapping.*warning" "$OUT/compile.log" | head -20 || true

echo "== running $BIN"
nice -n 19 "$OUT/$BIN" "$OUT/synth_xyz.f32"

echo "== independent voxel count (numpy) of the synthetic overflow cloud"
python3 "$ROOT/evaluation/tests/voxel_count_check.py" --raw "$OUT/synth_xyz.f32" --leaf 0.1 \
    --expect "$(cat "$OUT/synth_xyz.f32.count")"
