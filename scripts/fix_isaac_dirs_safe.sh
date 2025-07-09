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

  if ! stat_out=$(stat -c "%U %G %a" "$path" 2>/dev/null); then
    echo "🚫 Cannot access $path (permission denied or not found)"
    return
  fi

  read -r owner group perms <<< "$stat_out"

  # Normalize mode to 4-digit form
  [[ "${#perms}" -lt 4 ]] && perms="0$perms"

  # Compute expected string
  local expected="$TARGET_USER:$TARGET_GROUP $TARGET_MODE"
  local actual="$owner:$group $perms"

  # Check all conditions
  if [[ "$owner" != "$TARGET_USER" || "$group" != "$TARGET_GROUP" || "$perms" != "$TARGET_MODE" ]]; then
    echo "❗ $path: [$actual] → [$expected]"
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
  find "$root" -type d -print0 2>/dev/null | while IFS= read -r -d '' dir; do
    check_and_fix "$dir"
  done
  check_and_fix "$root"
done

if ! $FIX; then
  echo "✅ Done. Run with --fix to apply changes."
fi
