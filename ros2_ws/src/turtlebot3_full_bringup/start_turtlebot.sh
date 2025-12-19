#!/usr/bin/env bash

#umgebung
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

ros2 launch turtlebot3_full_bringup full_bringup.launch.py & LAUNCH_PID=$!

sleep 10

#initialpose setzen

ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: map
pose:
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  covariance: [
    0.25, 0, 0, 0, 0, 0,
    0, 0.25, 0, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 0.0685389
  ]"

  wait $LAUNCH_PID