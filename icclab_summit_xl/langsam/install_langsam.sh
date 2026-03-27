#!/usr/bin/env bash
# install_langsam.sh
#
# Installs lang-segment-anything with the patches required for icclab_summit_xl.
#
# What this script does:
#   1. Clones lang-segment-anything into ~/rap/lang-segment-anything
#   2. Creates a Python 3.12 venv with uv (with system site-packages for ROS access)
#   3. Installs PyTorch with CUDA support
#   4. Installs lang-segment-anything and its dependencies
#   5. Applies two source patches:
#        - lang_sam/lang_sam.py  : adds gdino_model_id parameter to LangSAM.__init__
#        - lang_sam/models/gdino.py : propagates model_id through build_model and
#                                     switches default to grounding-dino-tiny
#          (the committed upstream fix changed the default from base → tiny for
#           compatibility with Ubuntu 24.04 / ROS Jazzy)
#   6. Copies server_ros.py (ROS-aware LitServe server with JSON endpoint) into
#      lang_sam/ so the venv install picks it up
#
# Usage:
#   bash install_langsam.sh
#
# Re-running is safe: the venv and repo are reused if they already exist.

set -euo pipefail

REPO_DIR="$HOME/rap/lang-segment-anything"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATCHES_DIR="$SCRIPT_DIR/patches"
SERVER_FILE="$SCRIPT_DIR/server_ros.py"

# ---------------------------------------------------------------------------
# 1. Clone or update the repo
# ---------------------------------------------------------------------------
if [ ! -d "$REPO_DIR/.git" ]; then
    echo ">>> Cloning lang-segment-anything..."
    mkdir -p "$HOME/rap"
    git clone https://github.com/luca-medeiros/lang-segment-anything.git "$REPO_DIR"
else
    echo ">>> lang-segment-anything already cloned at $REPO_DIR, skipping clone."
fi

cd "$REPO_DIR"

# Apply the committed upstream fix (grounding-dino-base → tiny for Ubuntu 24.04)
# This was committed locally but not pushed. We cherry-pick the specific change
# by ensuring the repo is at a known good state; since we apply patches below
# this step is implicit.

# ---------------------------------------------------------------------------
# 2. Install uv if not present
# ---------------------------------------------------------------------------
if ! command -v uv &>/dev/null; then
    echo ">>> Installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    # shellcheck disable=SC1090
    source "$HOME/.local/bin/env" 2>/dev/null || export PATH="$HOME/.local/bin:$PATH"
fi
echo ">>> uv version: $(uv --version)"

# ---------------------------------------------------------------------------
# 3. Create venv if not present
# ---------------------------------------------------------------------------
if [ ! -d "$REPO_DIR/.venv" ]; then
    echo ">>> Creating Python 3.12 venv with system site-packages..."
    uv venv --python 3.12 --system-site-packages "$REPO_DIR/.venv"
else
    echo ">>> Venv already exists at $REPO_DIR/.venv, skipping creation."
fi

# ---------------------------------------------------------------------------
# 4. Install PyTorch with CUDA
# ---------------------------------------------------------------------------
# Detect CUDA version from nvcc or nvidia-smi to pick the right index URL.
CUDA_VERSION=""
if command -v nvcc &>/dev/null; then
    CUDA_VERSION=$(nvcc --version | grep -oP 'release \K[0-9]+\.[0-9]+')
elif command -v nvidia-smi &>/dev/null; then
    CUDA_VERSION=$(nvidia-smi | grep -oP 'CUDA Version: \K[0-9]+\.[0-9]+')
fi

if [ -z "$CUDA_VERSION" ]; then
    echo ">>> WARNING: Could not detect CUDA version. Defaulting to cu128."
    echo "            Edit this script if your CUDA version differs."
    CUDA_TAG="cu128"
else
    # Convert e.g. "12.8" → "cu128"
    CUDA_TAG="cu$(echo "$CUDA_VERSION" | tr -d '.')"
    echo ">>> Detected CUDA $CUDA_VERSION → using index tag $CUDA_TAG"
fi

TORCH_INDEX="https://download.pytorch.org/whl/${CUDA_TAG}"

echo ">>> Installing PyTorch from $TORCH_INDEX ..."
uv pip install --python "$REPO_DIR/.venv/bin/python" \
    torch torchvision \
    --index-url "$TORCH_INDEX"

# ---------------------------------------------------------------------------
# 5. Install lang-segment-anything
# ---------------------------------------------------------------------------
echo ">>> Installing lang-segment-anything (editable)..."
uv pip install --python "$REPO_DIR/.venv/bin/python" -e "$REPO_DIR"

# ---------------------------------------------------------------------------
# 6. Apply patches
# ---------------------------------------------------------------------------
echo ">>> Applying icclab patches..."

# lang_sam/lang_sam.py — adds gdino_model_id parameter
cp "$PATCHES_DIR/lang_sam.py" "$REPO_DIR/lang_sam/lang_sam.py"
echo "    Patched lang_sam/lang_sam.py"

# lang_sam/models/gdino.py — propagates model_id, keeps grounding-dino-tiny default
cp "$PATCHES_DIR/gdino.py" "$REPO_DIR/lang_sam/models/gdino.py"
echo "    Patched lang_sam/models/gdino.py"

# ---------------------------------------------------------------------------
# 7. Install server_ros.py
# ---------------------------------------------------------------------------
echo ">>> Installing server_ros.py..."
cp "$SERVER_FILE" "$REPO_DIR/lang_sam/server_ros.py"
echo "    Copied lang_sam/server_ros.py"

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
echo ""
echo "=== Installation complete ==="
echo ""
echo "Verify GPU support:"
echo "  source $REPO_DIR/.venv/bin/activate"
echo "  python -c \"import torch; print('CUDA available:', torch.cuda.is_available())\""
echo ""
echo "Start the ROS segmentation server:"
echo "  cd $REPO_DIR"
echo "  source .venv/bin/activate"
echo "  python3 -m lang_sam.server_ros"
echo ""
echo "Then in a separate terminal, launch the ROS node:"
echo "  ros2 launch icclab_summit_xl segmentation_remote.launch.py server_url:=http://localhost:8001"
