# Path Planning Algorithms

## Overview

**Path planning** finds collision-free routes from the robot's current position to a goal. Nav2 provides multiple planning algorithms optimized for different scenarios: global planners (A*, Dijkstra) for long-range paths and local planners (DWA, TEB) for real-time obstacle avoidance.

**What You'll Learn**:
- Global planning with A* and Smac Planner
- Local planning with DWA (Dynamic Window Approach)
- TEB (Timed Elastic Band) for smooth trajectories
- Configure planners for different robot types
- Tune planner parameters for performance

**Prerequisites**:
- Map Building (previous lesson)
- Understanding of graph search algorithms
- ROS 2 navigation concepts

**Estimated Time**: 3 hours

---

## Learning Objectives

1. Understand global vs. local planning
2. Configure A* for optimal path finding
3. Use DWA for dynamic obstacle avoidance
4. Apply TEB for smooth humanoid navigation
5. Tune planner parameters for speed/safety tradeoffs

---

## Part 1: Planning Architecture

### Two-Layer Planning

```
Goal Position
    ↓
Global Planner (A*, Smac) → Global Path (waypoints)
    ↓
Local Planner (DWA, TEB) → Velocity Commands
    ↓
Robot Controllers → Motor Commands
```

**Global Planner**: Computes path on static map (ignores dynamic obstacles)
**Local Planner**: Follows global path while avoiding real-time obstacles

---

## Part 2: Global Planning with A*

### A* Algorithm

**Cost Function**: `f(n) = g(n) + h(n)`
- `g(n)`: Cost from start to node n
- `h(n)`: Heuristic (estimated cost from n to goal)

### Configuration

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ['GridBased']
    GridBased:
      plugin: nav2_navfn_planner/NavfnPlanner
      tolerance: 0.5
      use_astar: true
      allow_unknown: true
```

### Launch Global Planner

```bash
ros2 launch nav2_bringup navigation_launch.py \
  map:=~/maps/warehouse_map.yaml \
  use_sim_time:=true
```

---

## Part 3: Local Planning with DWA

### DWA (Dynamic Window Approach)

**Concept**: Sample velocity space, simulate trajectories, choose best

**Parameters**:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ['FollowPath']
    FollowPath:
      plugin: dwb_core::DWBLocalPlanner
      max_vel_x: 0.5
      min_vel_x: -0.2
      max_vel_theta: 1.0
      acc_lim_x: 2.5
      acc_lim_theta: 3.2

      # Trajectory scoring
      path_distance_bias: 32.0
      goal_distance_bias: 24.0
      occdist_scale: 0.01
```

**Tuning Tips**:
- Increase `path_distance_bias` for tighter path following
- Increase `occdist_scale` for more cautious obstacle avoidance
- Reduce `max_vel_x` for safer navigation in crowded spaces

---

## Part 4: TEB Planner for Smooth Motion

### Timed Elastic Band

**Best for**: Humanoid robots, smooth trajectories, time-optimal paths

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ['FollowPath']
    FollowPath:
      plugin: teb_local_planner::TebLocalPlannerROS

      # Trajectory settings
      teb_autosize: true
      dt_ref: 0.3
      dt_hysteresis: 0.1
      min_samples: 3

      # Robot kinematics
      max_vel_x: 0.4
      max_vel_theta: 0.3
      acc_lim_x: 0.5
      acc_lim_theta: 0.5

      # Goal tolerance
      xy_goal_tolerance: 0.2
      yaw_goal_tolerance: 0.1
```

---

## Part 5: Testing in Isaac Sim

### Send Navigation Goal

**Method 1: RViz**
1. Click "2D Nav Goal" button
2. Click target position on map
3. Drag to set orientation

**Method 2: Python**

```python
from geometry_msgs.msg import PoseStamped

def send_nav_goal(x, y, yaw):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.z = math.sin(yaw / 2)
    goal.pose.orientation.w = math.cos(yaw / 2)

    nav_goal_pub.publish(goal)
```

### Visualize in RViz

Add displays:
- **Path** → `/plan` (global path)
- **Path** → `/local_plan` (DWA trajectory)
- **Marker** → `/goal_pose` (target)

---

## Part 6: Performance Comparison

| Planner | Speed | Smoothness | Obstacle Avoidance | Best For |
|---------|-------|------------|--------------------|----------|
| **A* (NavFn)** | Fast | Angular | Static obstacles | Simple robots |
| **Smac Hybrid** | Medium | Smooth | Static obstacles | Car-like robots |
| **DWA** | Fast | Moderate | Dynamic obstacles | Differential drive |
| **TEB** | Medium | Very Smooth | Dynamic obstacles | Humanoids |

---

## Summary

✅ Configured global planning with A*
✅ Implemented local planning with DWA
✅ Applied TEB for smooth humanoid motion
✅ Tuned parameters for navigation performance

**Next**: Obstacle avoidance and costmap configuration.

---

## Resources

- **Nav2 Planners**: https://navigation.ros.org/plugins/index.html
- **DWA Paper**: https://ieeexplore.ieee.org/document/580977
- **TEB Planner**: http://wiki.ros.org/teb_local_planner
