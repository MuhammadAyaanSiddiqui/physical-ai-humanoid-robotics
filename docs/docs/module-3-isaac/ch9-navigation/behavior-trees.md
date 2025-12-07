# Behavior Trees for Navigation

## Overview

**Behavior Trees (BTs)** provide flexible navigation logic in Nav2. Instead of hardcoded sequences, BTs allow you to define custom decision-making: "Try path A, if blocked try path B, if both fail perform recovery." This enables robust autonomous navigation that adapts to changing environments.

**What You'll Learn**:
- Understand behavior tree structure and nodes
- Read and modify Nav2 default behavior tree XML
- Create custom navigation behaviors
- Add conditional logic for complex scenarios
- Debug behavior tree execution

**Prerequisites**:
- Completed Chapter 9 (Map Building, Path Planning, Obstacle Avoidance)
- Understanding of decision trees and state machines

**Estimated Time**: 2-3 hours

---

## Learning Objectives

1. Understand BT concepts (actions, conditions, decorators, control nodes)
2. Interpret Nav2 default behavior tree
3. Modify BT XML for custom behaviors
4. Add retry logic and fallback strategies
5. Visualize and debug BT execution with Groot

---

## Part 1: Behavior Tree Fundamentals

### Node Types

| Type | Symbol | Description | Example |
|------|--------|-------------|---------|
| **Action** | Rectangle | Executes a task | ComputePathToPose |
| **Condition** | Ellipse | Checks a state | IsStuck |
| **Sequence** | → | Execute children in order, stop on failure | Plan → Follow → Success |
| **Fallback** | ? | Try children until one succeeds | TryPathA ? TryPathB ? Recover |
| **Decorator** | Diamond | Modifies child behavior | Retry(3x) |

### Execution Flow

```
Sequence (→)
├── ComputePathToPose → If fails, entire sequence fails
├── FollowPath → If succeeds, continue
└── Success → Mark navigation complete
```

```
Fallback (?)
├── ComputePathToPose ? If succeeds, done
└── RecoveryBehavior → If path fails, try recovery
```

---

## Part 2: Nav2 Default Behavior Tree

### Location

`/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning.xml`

### Simplified Structure

```xml
<root>
  <BehaviorTree ID="MainTree">
    <RecoveryNode>
      <Sequence>
        <!-- Try to navigate -->
        <ComputePathToPose goal="{goal}" path="{path}"/>
        <FollowPath path="{path}"/>
      </Sequence>

      <!-- Fallback: If navigation fails, try recovery -->
      <Fallback>
        <Sequence>
          <ClearCostmapExceptRegion/>
          <ComputePathToPose goal="{goal}" path="{path}"/>
        </Sequence>

        <Sequence>
          <Spin/>
          <ComputePathToPose goal="{goal}" path="{path}"/>
        </Sequence>

        <Sequence>
          <BackUp/>
          <ComputePathToPose goal="{goal}" path="{path}"/>
        </Sequence>

        <Wait duration="5"/>
      </Fallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

**Flow**:
1. Compute path and follow it
2. If fails → Clear costmap artifacts, recompute
3. If still fails → Spin 360°, recompute
4. If still fails → Backup, recompute
5. If all fail → Wait 5 seconds, retry

---

## Part 3: Custom Behavior Tree

### Example: Multi-Goal Waypoint Navigation

**Scenario**: Visit waypoints A, B, C in sequence, with recovery at each

```xml
<root>
  <BehaviorTree ID="WaypointNav">
    <Sequence>
      <!-- Waypoint A -->
      <RecoveryNode>
        <Sequence>
          <ComputePathToPose goal="{waypoint_a}" path="{path}"/>
          <FollowPath path="{path}"/>
        </Sequence>
        <FallbackRecovery/>
      </RecoveryNode>

      <!-- Waypoint B -->
      <RecoveryNode>
        <Sequence>
          <ComputePathToPose goal="{waypoint_b}" path="{path}"/>
          <FollowPath path="{path}"/>
        </Sequence>
        <FallbackRecovery/>
      </RecoveryNode>

      <!-- Waypoint C -->
      <RecoveryNode>
        <Sequence>
          <ComputePathToPose goal="{waypoint_c}" path="{path}"/>
          <FollowPath path="{path}"/>
        </Sequence>
        <FallbackRecovery/>
      </RecoveryNode>
    </Sequence>
  </BehaviorTree>

  <!-- Reusable recovery subtree -->
  <BehaviorTree ID="FallbackRecovery">
    <Fallback>
      <ClearCostmapExceptRegion/>
      <Spin/>
      <BackUp/>
      <Wait duration="5"/>
    </Fallback>
  </BehaviorTree>
</root>
```

### Load Custom BT

```bash
ros2 launch nav2_bringup navigation_launch.py \
  default_bt_xml_filename:=/path/to/custom_bt.xml
```

---

## Part 4: Advanced BT Nodes

### RetryUntilSuccessful Decorator

```xml
<RetryUntilSuccessful num_attempts="3">
  <ComputePathToPose goal="{goal}" path="{path}"/>
</RetryUntilSuccessful>
```

**Effect**: Retry path planning up to 3 times before failing

### RateController Decorator

```xml
<RateController hz="1.0">
  <ComputePathToPose goal="{goal}" path="{path}"/>
</RateController>
```

**Effect**: Limit replanning to 1 Hz (prevent excessive computation)

### Condition Nodes

```xml
<IsBatteryLow/>  <!-- Check battery voltage -->
<IsStuck/>       <!-- Check if robot hasn't moved in 5 seconds -->
<GoalReached/>   <!-- Check if within goal tolerance -->
```

**Example**: Battery-aware navigation

```xml
<Sequence>
  <IsBatteryLow/>  <!-- If battery is low -->
  <ComputePathToPose goal="{charging_station}"/>  <!-- Go to charger -->
</Sequence>
```

---

## Part 5: Behavior Tree Variables

### Blackboard (Shared Memory)

BTs use a **blackboard** to share data between nodes:

```xml
<ComputePathToPose goal="{goal}" path="{path}"/>
<!-- Writes computed path to blackboard variable {path} -->

<FollowPath path="{path}"/>
<!-- Reads {path} from blackboard -->
```

### Setting Input Variables

```python
from nav2_msgs.action import NavigateToPose

# Set goal position (written to blackboard as {goal})
goal = NavigateToPose.Goal()
goal.pose.header.frame_id = 'map'
goal.pose.pose.position.x = 5.0
goal.pose.pose.position.y = 3.0
```

---

## Part 6: Debugging with Groot

### Install Groot (BT Visualizer)

```bash
sudo snap install groot
```

### Connect to Nav2

```bash
# Terminal 1: Launch Nav2 with BT logging
ros2 launch nav2_bringup navigation_launch.py \
  params_file:=nav2_params.yaml \
  use_sim_time:=true

# Terminal 2: Launch Groot
groot
```

**In Groot**:
1. **Monitor** tab → Connect to `localhost:1667`
2. Load behavior tree XML
3. Watch nodes execute in real-time (green=success, red=failure)

### Common Debugging

**Issue**: FollowPath always fails
**Debug**: Check blackboard → Is `{path}` populated?

**Issue**: Infinite recovery loop
**Debug**: Groot shows Spin → BackUp → Spin repeating
**Solution**: Add `num_attempts` limit to RecoveryNode

---

## Part 7: Practical Example

### Custom BT: "Explore then Return to Base"

```xml
<root>
  <BehaviorTree ID="ExploreAndReturn">
    <Sequence>
      <!-- Phase 1: Explore waypoints -->
      <ForEach items="{waypoints}" item="{current_waypoint}">
        <Sequence>
          <ComputePathToPose goal="{current_waypoint}" path="{path}"/>
          <FollowPath path="{path}"/>
        </Sequence>
      </ForEach>

      <!-- Phase 2: Return to base -->
      <Sequence>
        <ComputePathToPose goal="{home_base}" path="{path}"/>
        <FollowPath path="{path}"/>
      </Sequence>
    </Sequence>
  </BehaviorTree>
</root>
```

**Python Code** (set waypoints):

```python
waypoints = [
    create_pose(5.0, 3.0),
    create_pose(8.0, 7.0),
    create_pose(2.0, 9.0)
]

# Send to blackboard
blackboard['waypoints'] = waypoints
blackboard['home_base'] = create_pose(0.0, 0.0)
```

---

## Summary

✅ Understood behavior tree structure and nodes
✅ Modified Nav2 default BT for custom logic
✅ Added retry and fallback strategies
✅ Debugged BT execution with Groot
✅ Created exploration behavior tree

**Congratulations!** Chapter 9 (Autonomous Navigation) complete!

**Next Chapter**: Reinforcement Learning for policy training in Isaac Sim.

---

## Resources

- **Behavior Trees Overview**: https://www.behaviortree.dev/
- **Nav2 BT Nodes**: https://navigation.ros.org/behavior_trees/index.html
- **Groot**: https://github.com/BehaviorTree/Groot
- **BT.CPP**: https://www.behaviortree.dev/ (library used by Nav2)
