#!/bin/bash

# Kill all ign and gz processes if they exist
pkill ign
pkill gz

# Wait a moment to ensure processes are terminated
sleep 2

# Run the ROS 2 launch file with the specified arguments
ros2 launch diffbot turtlebot3_world.launch.py use_sim_time:=true

