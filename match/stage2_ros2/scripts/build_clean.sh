#!/usr/bin/env bash
# build_clean.sh —— 强制重编 rmtc_stage2_tm，专治“头文件/源码不同步 +
# 旧 object 时间戳更新”导致的 undefined reference（例如
# buildHandshakeRequestBody(int, const std::string&)）。
#
# 用法（在 ROS 工作空间的 src/rmtc_stage2_tm 下）：
#   bash scripts/build_clean.sh
#
# 前提：<ws>/src/rmtc_stage2_tm 是标准 colcon 布局；
# 若包名/目录不同，直接按 README §3.1 的手动命令做。
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pkg_dir="$(cd "${script_dir}/.." && pwd)"
ws_dir="$(cd "${pkg_dir}/../.." && pwd)"

if [[ ! -d "${ws_dir}/src" ]]; then
  echo "ERROR: 未识别工作空间布局：${ws_dir}/src 不存在" >&2
  echo "       请从 <ws>/src/rmtc_stage2_tm/scripts 运行本脚本" >&2
  exit 2
fi

echo "[1/4] touch 源文件，避免 tar 保留的旧 mtime 让 make 跳过重编"
find "${pkg_dir}/include" "${pkg_dir}/src" "${pkg_dir}/test" \
  -type f \( -name '*.cpp' -o -name '*.h' \) -exec touch {} +

echo "[2/4] 只删除本包的 build/install 产物"
rm -rf "${ws_dir}/build/rmtc_stage2_tm" "${ws_dir}/install/rmtc_stage2_tm"

echo "[3/4] colcon build"
# ROS 的 setup.bash 内部会引用未定义变量，source 时临时关掉 -u
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
cd "${ws_dir}"
colcon build --packages-select rmtc_stage2_tm --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "[4/4] 校验 event_prompt.cpp.o 里的握手符号（应为 2 参数版本）"
obj="$(find "${ws_dir}/build/rmtc_stage2_tm" -name 'event_prompt.cpp.o' -print -quit)"
if [[ -n "${obj}" ]]; then
  nm -C "${obj}" | grep 'buildHandshakeRequestBody' || true
  if ! nm -C "${obj}" | grep -q 'buildHandshakeRequestBody(int, std::__cxx11::basic_string'; then
    echo "WARN: 没看到 2 参数符号；确认 src/rmtc/event_prompt.cpp 是 2 参数定义" >&2
    echo "      期望：std::string buildHandshakeRequestBody(int tick, const std::string& model)" >&2
  fi
else
  echo "WARN: 没找到 event_prompt.cpp.o，检查 colcon 输出" >&2
fi

echo "DONE"
