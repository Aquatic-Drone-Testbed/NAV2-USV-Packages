#!/bin/bash
#Script for setting up ros dependencies
colcon build --symlink-install
source install/setup.bash