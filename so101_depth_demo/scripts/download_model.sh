#!/bin/bash
# Download the Depth Anything V2 Small ONNX model for the CPU demo.
#
# Unlike snap-twin/scripts/download_model.sh, this keeps the ONNX file as-is —
# no TensorRT engine conversion is needed for the CPU / ONNX Runtime path.
#
# Usage:
#   scripts/download_model.sh [DEST_DIR]
# Default DEST_DIR: $HOME/models

set -euo pipefail

DEST_DIR="${1:-$HOME/models}"
DEST="${DEST_DIR}/depth_anything_v2_small.onnx"

mkdir -p "${DEST_DIR}"

# On Ubuntu 24.04 the system Python is "externally managed" (PEP 668), so a bare
# `pip3 install` fails unless you are inside a virtual environment. Prefer uv (if
# a uv/venv environment is sourced) and otherwise fall back to the active venv's
# python. As a last resort, allow the user to opt into --break-system-packages.
PY="${PYTHON:-python3}"
if [[ -n "${VIRTUAL_ENV:-}" && -x "${VIRTUAL_ENV}/bin/python" ]]; then
  PY="${VIRTUAL_ENV}/bin/python"
fi

echo "Using Python interpreter: ${PY}"

install_pkg() {
  # 1) uv (works with the sourced uv/venv environment)
  if command -v uv >/dev/null 2>&1; then
    echo "Installing huggingface_hub via uv..."
    if [[ -n "${VIRTUAL_ENV:-}" ]]; then
      uv pip install --quiet huggingface_hub && return 0
    else
      # No active venv: uv needs an explicit target python.
      uv pip install --quiet --python "${PY}" huggingface_hub && return 0
    fi
  fi

  # 2) Plain pip inside an active virtual environment
  if [[ -n "${VIRTUAL_ENV:-}" ]]; then
    echo "Installing huggingface_hub via pip (venv)..."
    "${PY}" -m pip install --quiet huggingface_hub && return 0
  fi

  # 3) Already importable? Nothing to do.
  if "${PY}" -c "import huggingface_hub" >/dev/null 2>&1; then
    echo "huggingface_hub already available."
    return 0
  fi

  # 4) Opt-in escape hatch for system Python (PEP 668).
  if [[ "${ALLOW_BREAK_SYSTEM_PACKAGES:-0}" == "1" ]]; then
    echo "Installing huggingface_hub via pip (--break-system-packages)..."
    "${PY}" -m pip install --quiet --break-system-packages huggingface_hub && return 0
  fi

  echo "ERROR: could not install huggingface_hub." >&2
  echo "On Ubuntu 24.04 the system Python is externally managed (PEP 668)." >&2
  echo "Activate your uv/venv environment first, e.g.:" >&2
  echo "    source .venv/bin/activate    # or your uv env" >&2
  echo "then re-run this script. To force the system Python instead, run:" >&2
  echo "    ALLOW_BREAK_SYSTEM_PACKAGES=1 $0 ${DEST_DIR}" >&2
  return 1
}

# Only install if huggingface_hub isn't already importable.
if ! "${PY}" -c "import huggingface_hub" >/dev/null 2>&1; then
  install_pkg
fi

echo "Downloading Depth Anything V2 Small ONNX..."
"${PY}" - "$DEST" << 'EOF'
import os
import shutil
import sys

from huggingface_hub import hf_hub_download

dest = sys.argv[1]
path = hf_hub_download(
    repo_id="onnx-community/depth-anything-v2-small",
    filename="onnx/model.onnx",
    cache_dir="/tmp/da_cache",
)
shutil.copy(path, dest)
print(f"Saved to {dest} ({os.path.getsize(dest) // 1024 // 1024} MB)")
EOF

echo ""
echo "Done. Use it with:"
echo "  ros2 launch so101_depth_demo depth_demo.launch.py model_path:=${DEST}"
