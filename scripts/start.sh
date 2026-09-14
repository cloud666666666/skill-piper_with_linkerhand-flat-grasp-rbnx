#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

if [[ ! -d "$PKG/rbnx-build/codegen" ]]; then
    echo "[flat_grasp/start] ERR: codegen output missing — run scripts/build.sh" >&2
    exit 2
fi

if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PKG:${PYTHONPATH:-}"
else
    export PYTHONPATH="/root/workspace/refs/robonix/pylib/robonix-api:$PKG:${PYTHONPATH:-}"
fi
export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"

exec python3 -u -m flat_grasp.main
