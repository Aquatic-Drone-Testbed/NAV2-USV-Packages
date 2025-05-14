#!/bin/bash
# To gain permission: chmod +x scripts/build-all.sh
chmod +x ./scripts/*
./scripts/install-nav2.sh
./scripts/rosdep.sh
./scripts/colcon-build.sh
