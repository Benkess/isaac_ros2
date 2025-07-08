#!/bin/bash

set -e

MODE=$1  # "shell", "gui", or "headless"
ISAAC_SIF="/containers/isaac_ros2_humble.sif"
HOME_DIR="$HOME"

# Check mode argument
if [[ "$MODE" != "shell" && "$MODE" != "gui" && "$MODE" != "headless" ]]; then
    echo "Usage: $0 [shell|gui|headless]"
    exit 1
fi

# Common binds for cache/log persistence and projects
BIND_FLAGS="--nv --contain \
  --bind /var/cache/isaac/kit:/isaac-sim/kit/cache:rw \
  --bind /var/cache/isaac/ov:$HOME_DIR/.cache/ov:rw \
  --bind /var/cache/isaac/pip:$HOME_DIR/.cache/pip:rw \
  --bind /var/cache/isaac/glcache:$HOME_DIR/.cache/nvidia/GLCache:rw \
  --bind /var/cache/isaac/computecache:$HOME_DIR/.nv/ComputeCache:rw \
  --bind /var/cache/isaac/logs:$HOME_DIR/.nvidia-omniverse/logs:rw \
  --bind /var/cache/isaac/data:$HOME_DIR/.local/share/ov/data:rw \
  --bind /projects:$HOME_DIR/Documents:rw \
  --bind /persistent/isaac/asset_root:/persistent/isaac/asset_root:rw"

# Optional: add --cleanenv if you want an isolated environment
ENV_FLAGS="--env ACCEPT_EULA=Y --env PRIVACY_CONSENT=Y"

case "$MODE" in
  shell)
    echo "[*] Launching container shell..."
    exec apptainer shell $BIND_FLAGS $ENV_FLAGS "$ISAAC_SIF"
    ;;

  gui)
    echo "[*] Launching Isaac Sim with GUI..."
    exec apptainer exec $BIND_FLAGS $ENV_FLAGS \
      --bind /tmp/.X11-unix:/tmp/.X11-unix \
      --env DISPLAY=$DISPLAY \
      "$ISAAC_SIF" \
      /bin/bash -lc "cd /isaac-sim && ./isaac-sim.sh"
    ;;

  headless)
    echo "[*] Launching Isaac Sim in headless mode..."
    exec apptainer exec $BIND_FLAGS $ENV_FLAGS "$ISAAC_SIF" \
      /bin/bash -lc "cd /isaac-sim && ./runheadless.sh -v"
    ;;
esac
