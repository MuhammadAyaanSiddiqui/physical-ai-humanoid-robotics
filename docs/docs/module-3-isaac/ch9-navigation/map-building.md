# Map Building with SLAM

## Overview

**Map building** is the foundation of autonomous navigation. Using SLAM (Simultaneous Localization and Mapping), robots create 2D occupancy grid maps showing free space, obstacles, and unexplored areas. These maps enable path planning and autonomous navigation with Nav2.

**What You'll Learn**:
- Build 2D occupancy grid maps with SLAM
- Save and load maps for navigation
- Evaluate map quality and completeness
- Optimize mapping strategies for different environments
- Export maps for Nav2 autonomous navigation

**Prerequisites**:
- Completed Chapter 8 (VSLAM lesson)
- Understanding of ROS 2 tf frames
- Nav2 concepts (covered in this chapter)

**Estimated Time**: 2-3 hours

---

## Learning Objectives

1. Build 2D occupancy grid maps using SLAM Toolbox
2. Save maps in standard formats (PGM + YAML)
3. Load pre-built maps for localization
4. Evaluate map quality metrics
5. Handle dynamic environments and map updates

---

## Part 1: SLAM Toolbox Setup

### Installation

```bash
sudo apt install ros-humble-slam-toolbox
```

### Launch SLAM Toolbox

```bash
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  slam_params_file:=/path/to/slam_params.yaml
```

**Configuration** (`slam_params.yaml`):

```yaml
slam_toolbox:
  ros__parameters:
    # Solver parameters
    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT

    # Scan matching
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.2
    minimum_travel_heading: 0.2

    # Map parameters
    resolution: 0.05  # 5cm grid cells
    max_laser_range: 20.0

    # Loop closure
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
```

---

## Part 2: Building a Map in Isaac Sim

### Step 1: Setup Scene

1. Launch Isaac Sim
2. Load warehouse: `Simple_Warehouse/warehouse.usd`
3. Add Carter robot with LiDAR

### Step 2: Enable ROS 2 Bridge

```python
# Enable LiDAR topic
import omni.isaac.ros2_bridge as ros_bridge
ros_bridge.enable_topic('/scan')
ros_bridge.enable_topic('/odom')
```

### Step 3: Drive Robot and Build Map

```bash
# Terminal 1: SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# Terminal 2: Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3: RViz
rviz2
```

**RViz Setup**:
- Add **Map** display → Topic: `/map`
- Add **LaserScan** → Topic: `/scan`
- Add **RobotModel** → Description: Carter URDF

Drive the robot to explore the environment. Watch the map build in real-time.

---

## Part 3: Saving Maps

### Save Map Command

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/warehouse_map
```

**Output Files**:
- `warehouse_map.pgm` - Image (grayscale: white=free, black=obstacle, gray=unknown)
- `warehouse_map.yaml` - Metadata

**YAML Contents**:

```yaml
image: warehouse_map.pgm
resolution: 0.05  # meters/pixel
origin: [-10.0, -10.0, 0.0]  # Map origin (meters)
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

---

## Part 4: Loading Maps for Localization

### Map Server

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=~/maps/warehouse_map.yaml
```

### AMCL Localization

```bash
ros2 launch nav2_bringup localization_launch.py \
  map:=~/maps/warehouse_map.yaml \
  use_sim_time:=true
```

---

## Part 5: Map Quality Evaluation

### Metrics

| Metric | Good Value | Poor Value |
|--------|------------|------------|
| **Coverage** | >95% explored | &lt;80% explored |
| **Alignment** | Straight walls | Wavy/duplicated walls |
| **Loop Closure** | Consistent when revisiting areas | Large jumps/discontinuities |

### Validation Checklist

- [ ] All rooms/areas explored
- [ ] Walls are straight and aligned
- [ ] No ghost obstacles (artifacts)
- [ ] Doorways clearly marked
- [ ] Loop closures detected (check SLAM logs)

---

## Summary

✅ Built 2D occupancy grid maps with SLAM Toolbox
✅ Saved maps in PGM + YAML format
✅ Loaded maps for localization with AMCL
✅ Evaluated map quality metrics

**Next**: Path planning algorithms for autonomous navigation.

---

## Resources

- **SLAM Toolbox**: https://github.com/SteveMacenski/slam_toolbox
- **Nav2 Map Server**: https://navigation.ros.org/configuration/packages/configuring-map-server.html
