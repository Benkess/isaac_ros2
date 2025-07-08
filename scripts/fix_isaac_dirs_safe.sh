#!/bin/bash

# Safe permission fixer for Isaac Sim host bind directories

ROOT_DIRS=(
  "/var/cache/isaac"
  "/persistent/isaac"
)

TARGET_USER="root"
TARGET_GROUP="isaac"
TARGET_MODE="2775"
DRY_RUN=false
FIX=false

usage() {
  echo "Usage: $0 [--dry-run] [--fix]"
  echo "  --dry-run   Only show what would be fixed"
  echo "  --fix       Actually fix permissions and ownership"
  exit 1
}

# Parse arguments
while [[ "$#" -gt 0 ]]; do
  case "$1" in
    --dry-run) DRY_RUN=true ;;
    --fix) FIX=true ;;
    *) usage ;;
  esac
  shift
done

echo "🔍 Checking Isaac Sim directories..."

# Function to check and optionally fix a directory
check_and_fix() {
  local path="$1"
  local stat_out
  stat_out=$(stat -c "%U %G %a" "$path")
  read -r owner group mode <<< "$stat_out"

  # Normalize mode to 4-digit form
  [[ "${#mode}" -lt 4 ]] && mode="0$mode"

  if [[ "$owner" != "$TARGET_USER" || "$group" != "$TARGET_GROUP" || "$mode" != "$TARGET_MODE" ]]; then
    echo "❗ $path: [$owner:$group $mode] → [$TARGET_USER:$TARGET_GROUP $TARGET_MODE]"
    if $FIX; then
      sudo chown "$TARGET_USER:$TARGET_GROUP" "$path"
      sudo chmod "$TARGET_MODE" "$path"
      echo "✅ Fixed $path"
    fi
  fi
}

# Traverse directories
for root in "${ROOT_DIRS[@]}"; do
  if [[ ! -d "$root" ]]; then
    echo "⚠️  Skipping missing root: $root"
    continue
  fi
  find "$root" -type d -print0 | while IFS= read -r -d '' dir; do
    check_and_fix "$dir"
  done
done

if ! $FIX; then
  echo "✅ Done. Run with --fix to apply changes."
fi
