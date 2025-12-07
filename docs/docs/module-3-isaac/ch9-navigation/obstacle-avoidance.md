# Obstacle Avoidance & Costmaps

## Overview

**Costmaps** represent the environment as a grid where each cell has a cost (0=free, 255=obstacle). Nav2 uses layered costmaps to combine static maps, sensor data, and inflation zones for safe obstacle avoidance. Recovery behaviors handle situations when the robot gets stuck.

**What You'll Learn**:
- Configure costmap layers (static, obstacle, inflation)
- Integrate sensor data (LiDAR, depth cameras) into costmaps
- Set up recovery behaviors for stuck situations
- Tune inflation radius and cost scaling
- Handle dynamic obstacles

**Prerequisites**:
- Path Planning (previous lesson)
- Understanding of occupancy grids
- Sensor integration (LiDAR, cameras)

**Estimated Time**: 2-3 hours

---

## Learning Objectives

1. Configure multi-layer costmaps
2. Integrate real-time sensor data
3. Set inflation parameters for safety margins
4. Implement recovery behaviors
5. Handle dynamic obstacles effectively

---

## Part 1: Costmap Architecture

### Layered Costmap Structure

```
Final Costmap = Static Layer + Obstacle Layer + Inflation Layer
```

**Layers**:
1. **Static Layer**: From pre-built map (walls, fixed obstacles)
2. **Obstacle Layer**: From sensors (LiDAR, cameras) - dynamic
3. **Inflation Layer**: Safety margin around obstacles
4. **Voxel Layer** (optional): 3D obstacles for elevated sensors

---

## Part 2: Costmap Configuration

### Basic Setup

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05

      plugins: ['static_layer', 'obstacle_layer', 'inflation_layer']

      static_layer:
        plugin: nav2_costmap_2d::StaticLayer
        map_subscribe_transient_local: true

      obstacle_layer:
        plugin: nav2_costmap_2d::ObstacleLayer
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: LaserScan

      inflation_layer:
        plugin: nav2_costmap_2d::InflationLayer
        inflation_radius: 0.55
        cost_scaling_factor: 3.0
```

### Global vs. Local Costmap

| Aspect | Global Costmap | Local Costmap |
|--------|----------------|---------------|
| **Size** | Entire map | 3x3m around robot |
| **Update Rate** | Slow (1 Hz) | Fast (5 Hz) |
| **Use** | Global planning | Local planning |
| **Frame** | map | odom |

---

## Part 3: Sensor Integration

### LiDAR Integration

```yaml
observation_sources: scan
scan:
  topic: /scan
  sensor_frame: laser_frame
  observation_persistence: 0.0
  expected_update_rate: 0.0
  data_type: LaserScan
  min_obstacle_height: 0.0
  max_obstacle_height: 2.0
  obstacle_max_range: 2.5
  obstacle_min_range: 0.0
  raytrace_max_range: 3.0
  raytrace_min_range: 0.0
  clearing: true
  marking: true
```

### Depth Camera Integration

```yaml
observation_sources: scan depth_camera
depth_camera:
  topic: /camera/depth/points
  sensor_frame: camera_depth_frame
  data_type: PointCloud2
  min_obstacle_height: 0.1
  max_obstacle_height: 1.5
  obstacle_max_range: 2.0
  clearing: true
  marking: true
```

---

## Part 4: Inflation Parameters

### Inflation Radius

**Purpose**: Create safety margin around obstacles

```yaml
inflation_layer:
  inflation_radius: 0.55  # meters
  cost_scaling_factor: 3.0
```

**Effect**:
- `inflation_radius: 0.3` → Tight navigation, risk of collision
- `inflation_radius: 0.8` → Conservative, wider berth around obstacles

### Cost Scaling

**Formula**: `cost = 253 * exp(-cost_scaling_factor * distance)`

```yaml
cost_scaling_factor: 1.0  # Gradual cost increase
cost_scaling_factor: 10.0  # Sharp cost increase (more conservative)
```

---

## Part 5: Recovery Behaviors

### Configuration

```yaml
recoveries_server:
  ros__parameters:
    recovery_plugins: ['spin', 'backup', 'wait']

    spin:
      plugin: nav2_recoveries::Spin
      max_rotational_vel: 1.0
      min_rotational_vel: 0.4
      rotational_acc_lim: 3.2

    backup:
      plugin: nav2_recoveries::BackUp
      backup_dist: 0.3
      backup_speed: 0.05

    wait:
      plugin: nav2_recoveries::Wait
      wait_duration: 5
```

### Recovery Sequence

1. **Clear costmap** (remove sensor artifacts)
2. **Spin in place** (360° rotation to find opening)
3. **Backup** (reverse 30cm)
4. **Wait** (5 seconds for dynamic obstacles to move)
5. **Abort** (if all recovery attempts fail)

---

## Part 6: Dynamic Obstacles

### Handling Moving Objects

**Temporal Marking**:

```yaml
obstacle_layer:
  observation_persistence: 1.0  # Keep obstacles for 1 second
```

**Clearing vs. Marking**:
- **Marking**: Add obstacle to costmap (when sensor detects something)
- **Clearing**: Remove obstacle (when sensor reports free space)

```yaml
scan:
  clearing: true  # Clear obstacles not seen
  marking: true   # Mark detected obstacles
```

---

## Part 7: Testing in Isaac Sim

### Scenario: Navigate Around Dynamic Obstacles

1. Load warehouse scene with moving objects
2. Launch Nav2 with costmap configuration
3. Send navigation goal
4. Observe robot avoiding obstacles in real-time

**Visualization in RViz**:
- Add **Costmap** → `/local_costmap/costmap`
- Colors: Blue=free, Red=obstacle, Yellow=inflated

### Common Issues

**Issue**: Robot stops unexpectedly
**Cause**: Phantom obstacles in costmap
**Solution**: Lower `observation_persistence`, enable clearing

**Issue**: Robot collides with obstacles
**Cause**: Inflation radius too small
**Solution**: Increase `inflation_radius` to 0.7-0.8m

---

## Summary

✅ Configured layered costmaps (static, obstacle, inflation)
✅ Integrated LiDAR and depth cameras
✅ Tuned inflation for safety margins
✅ Implemented recovery behaviors
✅ Handled dynamic obstacles

**Next**: Behavior trees for navigation logic.

---

## Resources

- **Nav2 Costmaps**: https://navigation.ros.org/configuration/packages/configuring-costmaps.html
- **Recovery Behaviors**: https://navigation.ros.org/plugins/index.html#recovery
