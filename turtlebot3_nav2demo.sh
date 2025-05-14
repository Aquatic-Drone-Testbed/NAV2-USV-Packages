#!/bin/bash
echo "Running Nav2 Turtlebot3 demo..."
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle  # Iron and older only with Gazebo Classic
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models # Iron and older only with Gazebo Classic
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False