# SPDX-License-Identifier: Apache-2.0
# Build phase: rbnx codegen — proto+gRPC stubs for the gRPC upstreams, AND
# `--mcp` for the pydantic dataclasses the @mcp handlers are typed against.
#
# --mcp is NOT optional here: robonix_api's @provider.mcp decorator calls
# `input_cls.json_schema()` on the annotated request type (tool.py:make_shim).
# Protobuf messages have no such method, so typing the handler against
# `pick_pb2.Pick_Request` dies at import with a bare `AttributeError:
# json_schema` — before the package can even register with atlas.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[flat_grasp/build] clean: removing rbnx-build/"
    rm -rf rbnx-build
fi
mkdir -p rbnx-build/data

FLAGS=(--out-dir "$PKG/rbnx-build/codegen" --mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[flat_grasp/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

touch "$PKG/rbnx-build/.rbnx-built"
echo "[flat_grasp/build] done."
