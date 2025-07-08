#!/bin/bash

# Directories to check
BASE_DIRS=(
  /var/cache/isaac
  /persistent/isaac
)

REQUIRED_GROUP="isaac"
REQUIRED_MODE="2775"

echo "=== Isaac Sim Directory Permission Audit ==="

for BASE in "${BASE_DIRS[@]}"; do
  if [ ! -d "$BASE" ]; then
    echo "[ERROR] Missing base directory: $BASE"
    continue
  fi

  echo "-- Checking under $BASE --"
  find "$BASE" -type d | while read -r DIR; do
    # Get stats
    OWNER=$(stat -c "%U" "$DIR")
    GROUP=$(stat -c "%G" "$DIR")
    MODE=$(stat -c "%a" "$DIR")
    SGID=$(stat -c "%A" "$DIR" | cut -c6)  # Get group execute/SGID bit

    # Check conditions
    WARNED=false
    if [ "$OWNER" != "root" ]; then
      echo "[WARN] $DIR has owner '$OWNER' (expected: root)"
      WARNED=true
    fi
    if [ "$GROUP" != "$REQUIRED_GROUP" ]; then
      echo "[WARN] $DIR has group '$GROUP' (expected: $REQUIRED_GROUP)"
      WARNED=true
    fi
    if [ "$MODE" != "$REQUIRED_MODE" ]; then
      echo "[WARN] $DIR has mode '$MODE' (expected: $REQUIRED_MODE)"
      WARNED=true
    fi
    if [ "$SGID" != "s" ]; then
      echo "[WARN] $DIR missing setgid bit (expected: rwxrwsr-x)"
      WARNED=true
    fi

    if [ "$WARNED" = true ]; then
      echo "  → Suggested fix:"
      echo "    sudo chown root:$REQUIRED_GROUP '$DIR'"
      echo "    sudo chmod $REQUIRED_MODE '$DIR'"
      echo
    fi
  done
done
