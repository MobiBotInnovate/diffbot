#!/bin/bash

# Reset Gazebo simulation
ros2 service call /reset_simulation std_srvs/srv/Empty

# Reset Nav2 components
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap

# Reset SLAM Toolbox
ros2 service call /slam_toolbox/clear_changes slam_toolbox/srv/Clear

echo "Simulation has been reset."
