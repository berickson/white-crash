#!/bin/bash

distrobox enter humble -- /bin/bash << 'EOF'
echo "Sourcing ROS Humble..."
source /opt/ros/humble/setup.bash || { echo "Failed to source ROS Humble"; exit 1; }

echo "Building workspace..."
cd ~/Projects/wc_ws || { echo "Failed to enter workspace directory"; exit 1; }
colcon build || { echo "Failed to build workspace"; exit 1; }

echo "Sourcing workspace..."
source install/setup.bash || { echo "Failed to source workspace"; exit 1; }

echo "Starting Foxglove Bridge..."
exec ros2 run foxglove_bridge foxglove_bridge
EOF
