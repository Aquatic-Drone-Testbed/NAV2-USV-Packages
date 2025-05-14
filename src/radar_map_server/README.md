# radar_map_server

A ROS 2 package that publishes a radar-derived heatmap as a `nav_msgs/OccupancyGrid` on 
the `/map` topic. This occupancy grid is suitable for use with the Navigation2 (Nav2) stack 
without requiring PGM/YAML files.

---

## Overview

The `heatmap_publisher` node:

1. Loads a NumPy `.npy` file containing a two-dimensional array of confidence values.
2. Applies configurable thresholds to classify each cell as free (0), occupied (100), or unknown (−1).
3. Constructs a `nav_msgs/OccupancyGrid` message using specified resolution, origin, and coordinate frame.
4. Publishes the grid on `/map` with transient-local durability for Nav2 compatibility.

---

## Prerequisites

- ROS 2 Humble, Iron, or later (sourced).
- Colcon build tool.
- Python 3 and NumPy.
- A ROS 2 workspace (e.g. `~/ros2_ws`).

---

## Installation

1. Clone this repository into the workspace source directory:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> radar_map_server
   ```
2. Install ROS 2 dependencies if necessary:
   ```bash
   sudo apt update
   sudo apt install ros-<distro>-rclpy ros-<distro>-nav-msgs ros-<distro>-std-msgs
   ```
3. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select radar_map_server
   ```
4. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

---

## Usage

1. Generate or export a heatmap to a NumPy file (`.npy`).
2. Launch the publisher:
   ```bash
   ros2 launch radar_map_server heatmap_launch.py
   ```
3. Confirm that `/map` is an `OccupancyGrid`:
   ```bash
   ros2 topic echo /map --once
   ```

---

## Configuration

Parameters are defined in `launch/heatmap_launch.py` or may be overridden at runtime:

| Parameter             | Type   | Default                       | Description                                           |
|-----------------------|--------|-------------------------------|-------------------------------------------------------|
| `heatmap_file`        | string | `/home/you/maps/heatmap.npy`  | Path to the heatmap file.                             |
| `map_frame`           | string | `map`                         | Frame ID of the published grid.                       |
| `resolution`          | float  | `0.05`                        | Size of each cell in meters.                          |
| `origin_x`, `origin_y`| float  | `0.0`, `0.0`                  | Coordinates of the grid origin in the map frame.      |
| `free_thresh`         | float  | `10.0`                        | Values ≤ this threshold are free (0).                 |
| `occupied_thresh`     | float  | `90.0`                        | Values ≥ this threshold are occupied (100).           |

Runtime overrides example:

```bash
ros2 run radar_map_server heatmap_publisher \
  --ros-args -p heatmap_file:=/path/to/file.npy -p resolution:=0.1
```

---

## Integration with Nav2

Configure the global costmap in `nav2_params.yaml` to consume `/map`:

```yaml
global_costmap:
  ros__parameters:
    static_map: false
    map_topic: /map
    map_subscribe_transient_local: true
```

Launch Nav2:

```bash
ros2 launch nav2_bringup bringup_launch.py \
  map:=/map \
  params_file:=/path/to/nav2_params.yaml
```

---
