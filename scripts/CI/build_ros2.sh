#!/bin/bash
set -e

SIF_NAME="ros2_humble.sif"
DEF_FILE="ros2_humble.def"
BUILD_DIR="/localtmp/isaac_ros2"
OUTPUT_DIR="/containers"
GROUP="isaac"

echo "🧹 Cleaning up old image..."
sudo rm -f "$OUTPUT_DIR/$SIF_NAME"

echo "🛠️  Building new Apptainer SIF..."
cd "$BUILD_DIR"
apptainer build --fakeroot "$SIF_NAME" "$DEF_FILE"

echo "📦 Moving SIF to $OUTPUT_DIR"
sudo mv -f "$SIF_NAME" "$OUTPUT_DIR/"
sudo chown root:$GROUP "$OUTPUT_DIR/$SIF_NAME"
sudo chmod 2755 "$OUTPUT_DIR/$SIF_NAME"

echo "✅ Done."
