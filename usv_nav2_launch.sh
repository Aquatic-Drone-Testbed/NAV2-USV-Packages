#!/bin/bash

trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT

echo "Running Nav2 Stack..."
source install/setup.bash
ros2 launch radar_map_server heatmap_launch.py & #Launch map server
ros2 launch usv_nav2 pose_tf.launch.py & #Launch USV pose tf handler
ros2 run cmd_vel2thrust cmd_vel2thrust & #Launch thrust controller
ros2 launch nav2_bringup navigation_launch.py & #Launch nav2 navigation stack
ros2 launch nav2_bringup rviz_launch.py #Launch rviz with nav2 default settings
#ros2 run nav2_goal_setter nav2_goal_setter #Launch goal publisher system
# sleep infinity