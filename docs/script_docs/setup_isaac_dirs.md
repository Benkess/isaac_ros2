# Isaac Sim Directory Setup Guide for Apptainer

This guide provides an administrator script and instructions to prepare the host system for running Isaac Sim and ROS 2 inside an Apptainer container.

## Overview

Isaac Sim requires several persistent directories for caching, logging, assets, and project isolation. This script ensures all required paths exist and are writable by users in the `isaac` group.

## Admin Setup Script

### Filename: `setup_isaac_dirs.sh`

```bash
#!/bin/bash

set -e

# Optional: enable debug mode to chmod 2777
DEBUG_MODE=false

if [[ "$1" == "--debug" ]]; then
  DEBUG_MODE=true
  echo "[!] DEBUG MODE: Setting world-writable permissions (2777) for testing."
fi

# Create isaac group if missing
if ! getent group isaac >/dev/null; then
  echo "[*] Creating 'isaac' group..."
  groupadd isaac
else
  echo "[*] 'isaac' group already exists."
fi

# Required paths
CONTAINER_DIR="/containers"
CACHE_DIR="/var/cache/isaac"
PERSISTENT_DIR="/persistent/isaac/asset_root"
PROJECTS_DIR="/projects"

CACHE_SUBDIRS=(kit ov pip glcache computecache logs data)

echo "[*] Creating container root..."
mkdir -p "$CONTAINER_DIR"
chmod 755 "$CONTAINER_DIR"

echo "[*] Creating cache directories..."
for subdir in "${CACHE_SUBDIRS[@]}"; do
  mkdir -p "$CACHE_DIR/$subdir"
done

echo "[*] Creating persistent asset root..."
mkdir -p "$PERSISTENT_DIR"

echo "[*] Creating projects directory..."
mkdir -p "$PROJECTS_DIR"

echo "[*] Setting permissions and group ownership..."

if $DEBUG_MODE; then
  chmod -R 2777 "$CACHE_DIR" "$PERSISTENT_DIR" "$PROJECTS_DIR"
else
  chown -R root:isaac "$CACHE_DIR" "$PERSISTENT_DIR"
  chmod -R 2775 "$CACHE_DIR" "$PERSISTENT_DIR"
  chown -R root:isaac "$PROJECTS_DIR"
  chmod -R 2775 "$PROJECTS_DIR"
fi

echo "[\u2713] All directories created and permissions set."

echo ""
echo "\uD83D\uDC49 Reminder: Add users to the 'isaac' group with:"
echo "   sudo usermod -aG isaac <username>"
```

## How to Use

```bash
# Standard setup (root:isaac ownership, group writable)
sudo bash setup_isaac_dirs.sh

# Debug mode (world writable, use for testing only)
sudo bash setup_isaac_dirs.sh --debug
```

## Created Directory Tree

```
/containers
/var/cache/isaac/
  ├── kit
  ├── ov
  ├── pip
  ├── glcache
  ├── computecache
  ├── logs
  └── data
/persistent/isaac/asset_root
/projects
```

## Notes

* All users who need access must be added to the `isaac` group.
* Debug mode sets world-writable permissions (2777) to isolate permission issues.
* Restore standard permissions with the script run without `--debug`.
