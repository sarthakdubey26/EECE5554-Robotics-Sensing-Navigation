#!/bin/bash
# Quick script to record walking data

if [ $# -eq 0 ]; then
    echo "Usage: ./record_walk.sh <circle|square>"
    echo "Example: ./record_walk.sh circle"
    exit 1
fi

PATTERN=$1
OUTPUT_NAME="walk_${PATTERN}_$(date +%Y%m%d_%H%M%S)"

echo "=========================================="
echo "Recording: $PATTERN walk"
echo "Output: $OUTPUT_NAME"
echo "=========================================="
echo ""
echo "Instructions:"
echo "1. Start video recording on phone"
echo "2. Hold IMU with X-axis pointing forward"
echo "3. Keep IMU parallel to ground"
echo "4. Press ENTER when ready to start rosbag"
echo ""

read -p "Press ENTER to start recording..."

cd data
ros2 bag record -o $OUTPUT_NAME /imu

echo ""
echo "Recording saved to: data/$OUTPUT_NAME"
