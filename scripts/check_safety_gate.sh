#!/usr/bin/env bash
# 注意：不能用 set -u，ROS setup 脚本会引用未初始化的变量（如 AMENT_TRACE_SETUP_FILES）
set -eo pipefail

# 安全门验收：拉起 obstacle_stop 后跑 12 项场景检查（无需 Gazebo / Nav2）
# 用法：
#   scripts/check_safety_gate.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 未加载 ROS 环境时自动 source（优先 jazzy）
if [[ -z "${ROS_DISTRO:-}" ]]; then
  for distro in jazzy humble; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      # shellcheck disable=SC1091
      source "/opt/ros/${distro}/setup.bash"
      break
    fi
  done
fi

if [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${REPO_ROOT}/install/setup.bash"
fi

exec python3 "${SCRIPT_DIR}/check_safety_gate.py" "$@"
