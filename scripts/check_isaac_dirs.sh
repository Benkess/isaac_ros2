#!/bin/bash

# Directories to check
BASE_DIRS=(
  /var/cache/isaac
  /persistent/isaac
)

REQUIRED_GROUP="isaac"
REQUIRED_OWNER="root"
REQUIRED_MODE="2775"

echo "=== Isaac Sim Directory Permission Audit ==="

for BASE in "${BASE_DIRS[@]}"; do
  if [ ! -d "$BASE" ]; then
    echo "[ERROR] Missing base directory: $BASE"
    continue
  fi

  echo "-- Checking under $BASE --"
  find "$BASE" -type d 2>/dev/null | while read -r DIR; do
    OWNER=$(stat -c "%U" "$DIR")
    GROUP=$(stat -c "%G" "$DIR")
    MODE=$(stat -c "%a" "$DIR")

    SHOULD_WARN=0

    if [ "$OWNER" != "$REQUIRED_OWNER" ]; then
      echo "[WARN] $DIR has owner '$OWNER' (expected: $REQUIRED_OWNER)"
      SHOULD_WARN=1
    fi
    if [ "$GROUP" != "$REQUIRED_GROUP" ]; then
      echo "[WARN] $DIR has group '$GROUP' (expected: $REQUIRED_GROUP)"
      SHOULD_WARN=1
    fi
    if [ "$MODE" != "$REQUIRED_MODE" ]; then
      echo "[WARN] $DIR has mode '$MODE' (expected: $REQUIRED_MODE)"
      SHOULD_WARN=1
    fi

    if (( SHOULD_WARN == 1 )); then
      echo "  → Suggested fix:"
      echo "    sudo chown $REQUIRED_OWNER:$REQUIRED_GROUP '$DIR'"
      echo "    sudo chmod $REQUIRED_MODE '$DIR'"
      echo
    fi
  done
done
