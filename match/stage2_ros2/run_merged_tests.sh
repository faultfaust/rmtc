#!/usr/bin/env bash
# Build and run the regression suite for the merged event-dedup fix.
# Usage (on the robot): bash /ceshi/stage2_ros2_merged/run_merged_tests.sh
set -euo pipefail
cd "$(dirname "$0")"

source /opt/ros/humble/setup.bash
# 合并版改动涉及头文件；清掉本包旧 build/install，避免旧 object 干扰。
rm -rf build install log
colcon build --packages-select rmtc_stage2_tm --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

BIN=./install/rmtc_stage2_tm/lib/rmtc_stage2_tm
set +e
"$BIN/v12_check";            r1=$?
"$BIN/stage2_llm_e2e";       r2=$?
set -e

echo "----------------------------------------"
echo "v12_check=$r1  stage2_llm_e2e=$r2"
if [ "$r1" -ne 0 ] || [ "$r2" -ne 0 ]; then
  echo "REGRESSION FAILED"
  exit 1
fi
echo "REGRESSION PASS"
