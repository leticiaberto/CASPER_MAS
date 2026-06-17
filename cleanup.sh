#!/bin/bash
 
# Kill any live ROS2 / Gazebo / bridge processes first.
# Deleting shared-memory files while a process still holds them locked
# can leave things in a worse, half-cleaned state — so processes must
# die before we touch /dev/shm.
pkill -9 -f "ros2" 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "gzserver" 2>/dev/null
pkill -9 -f "parameter_bridge" 2>/dev/null
 
# Give the kernel a moment to release the locks before deleting files.
sleep 2
 
rm -f /tmp/gz_ros2_bridge.lock
 
# Remove stale Fast-DDS shared memory files and semaphores (can linger
# after a crash/SIGKILL and cause node discovery issues on the next run)
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -rf /dev/shm/sem.fastrtps_* 2>/dev/null
rm -rf /tmp/fastrtps_* 2>/dev/null
 
echo "Bridge and stale processes cleaned up."