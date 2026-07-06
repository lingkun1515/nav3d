#!/bin/bash
# Drive robot through hospital for SLAM mapping.
# Each segment: publish cmd_vel at given rate for N seconds, then stop briefly.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/install/setup.bash"

SPEED=0.3
TURN=0.3

drive() {
    local vx=$1 wz=$2 secs=$3
    echo "[$(date +%H:%M:%S)] Driving vx=$vx wz=$wz for ${secs}s"
    timeout $secs ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: $vx, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $wz}}" \
        --rate 10 2>/dev/null
    # Brief stop
    timeout 0.5 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
        --rate 20 2>/dev/null
    sleep 0.5
}

echo "[$(date +%H:%M:%S)] Starting hospital exploration"

# Phase 1: Ground floor - drive forward along corridor (y direction)
# From (0,0) drive forward ~10m (about 33s at 0.3 m/s)
drive 0.3 0.0 33

# Turn slightly left and continue
drive 0.3 0.1 10

# Continue forward toward ramp area
drive 0.3 0.0 20

# Turn toward ramp (at x=-1.71, y=16.37)
drive 0.0 -0.3 3
drive 0.3 0.0 15

# Phase 2: Up the ramp (drive forward, should ascend to z=3)
echo "[$(date +%H:%M:%S)] Going up the ramp"
drive 0.3 0.0 30

# Phase 3: Second floor exploration
echo "[$(date +%H:%M:%S)] Exploring second floor"
# Turn and explore
drive 0.0 0.3 5
drive 0.3 0.0 20
drive 0.0 -0.3 5
drive 0.3 0.0 20

# Turn around and explore other direction
drive 0.0 0.3 10
drive 0.3 0.0 30

# More exploration
drive 0.0 -0.3 5
drive 0.3 0.0 25

# Phase 4: Go back down ramp
echo "[$(date +%H:%M:%S)] Going back down ramp"
drive 0.0 3.14 2  # U-turn
drive 0.3 0.0 30

# Phase 5: Ground floor exploration - other direction
echo "[$(date +%H:%M:%S)] Exploring ground floor other side"
drive 0.0 0.3 5
drive 0.3 0.0 30
drive 0.0 -0.3 5
drive 0.3 0.0 30

# Explore negative y direction
drive 0.0 3.14 2  # U-turn
drive 0.3 0.0 40

# Final sweep
drive 0.0 0.3 5
drive 0.3 0.0 30
drive 0.0 -0.3 5
drive 0.3 0.0 30

echo "[$(date +%H:%M:%S)] Exploration complete!"
