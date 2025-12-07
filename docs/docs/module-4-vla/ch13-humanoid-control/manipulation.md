# Humanoid Manipulation and Grasping

## Learning Objectives

- Implement grasp planning for various object shapes
- Use force/torque feedback for stable grasping
- Perform pick-and-place operations
- Handle manipulation failures gracefully

**Estimated Time**: 3-4 hours

---

## Part 1: Grasp Planning

```python
import numpy as np

class GraspPlanner:
    def __init__(self):
        self.gripper_width = 0.08  # meters
        self.grasp_force = 10.0     # Newtons

    def plan_parallel_grasp(self, object_width, object_center):
        """
        Plan parallel jaw grasp for object

        Returns: (approach_pose, grasp_pose, close_distance)
        """
        # Approach from above
        approach_pose = object_center.copy()
        approach_pose[2] += 0.15  # 15cm above object

        # Grasp at object center
        grasp_pose = object_center.copy()

        # Close distance based on object width
        close_distance = object_width - 0.01  # Leave 1cm gap

        if close_distance > self.gripper_width:
            print(f"⚠️ Object too wide ({object_width:.3f}m > {self.gripper_width:.3f}m)")
            return None

        return {
            'approach': approach_pose,
            'grasp': grasp_pose,
            'close_distance': close_distance
        }

# Example
planner = GraspPlanner()

object_center = np.array([0.5, 0.2, 0.75])
object_width = 0.05  # 5cm cube

grasp_plan = planner.plan_parallel_grasp(object_width, object_center)

if grasp_plan:
    print(f"Approach: {grasp_plan['approach']}")
    print(f"Grasp: {grasp_plan['grasp']}")
    print(f"Close gripper to: {grasp_plan['close_distance']:.3f}m")
```

---

## Part 2: Force Control

```python
class ForceController:
    def __init__(self, desired_force=10.0, kp=0.1):
        self.desired_force = desired_force
        self.kp = kp

    def adjust_gripper(self, measured_force):
        """
        Adjust gripper based on force feedback

        Returns: velocity command for gripper
        """
        error = self.desired_force - measured_force
        velocity = self.kp * error

        # Safety limits
        if measured_force > 20.0:
            print("⚠️ Excessive force! Opening gripper")
            return -0.05  # Open slowly

        return velocity

# Example
force_ctrl = ForceController(desired_force=10.0)

# Gripper not squeezing hard enough
measured = 5.0
cmd = force_ctrl.adjust_gripper(measured)
print(f"Force error: {10.0 - measured:.1f}N → Gripper velocity: {cmd:.3f}")

# Gripper squeezing too hard
measured = 15.0
cmd = force_ctrl.adjust_gripper(measured)
print(f"Force error: {10.0 - measured:.1f}N → Gripper velocity: {cmd:.3f}")
```

---

## Part 3: Pick and Place Pipeline

```python
class PickAndPlaceController:
    def __init__(self):
        self.grasp_planner = GraspPlanner()
        self.force_controller = ForceController()
        self.state = "IDLE"

    async def execute_pick_and_place(self, object_info, place_location):
        """
        Complete pick and place operation

        States: IDLE → APPROACH → GRASP → LIFT → CARRY → PLACE → RELEASE → IDLE
        """
        # 1. Plan grasp
        self.state = "PLANNING"
        grasp_plan = self.grasp_planner.plan_parallel_grasp(
            object_info['width'],
            object_info['position']
        )

        if not grasp_plan:
            return False

        # 2. Approach object
        self.state = "APPROACH"
        await self.move_to(grasp_plan['approach'])

        # 3. Descend to grasp
        self.state = "GRASP"
        await self.move_to(grasp_plan['grasp'])

        # 4. Close gripper
        await self.close_gripper(grasp_plan['close_distance'])

        # 5. Lift object
        self.state = "LIFT"
        lift_pose = grasp_plan['grasp'].copy()
        lift_pose[2] += 0.10  # Lift 10cm
        await self.move_to(lift_pose)

        # 6. Carry to place location
        self.state = "CARRY"
        carry_pose = place_location.copy()
        carry_pose[2] += 0.10  # Stay 10cm above
        await self.move_to(carry_pose)

        # 7. Descend to place
        self.state = "PLACE"
        await self.move_to(place_location)

        # 8. Release object
        self.state = "RELEASE"
        await self.open_gripper()

        # 9. Retract
        self.state = "RETRACT"
        retract_pose = place_location.copy()
        retract_pose[2] += 0.10
        await self.move_to(retract_pose)

        self.state = "IDLE"
        return True

    async def move_to(self, target_pose):
        """Placeholder for motion execution"""
        print(f"Moving to {target_pose}")
        # In real system: call MoveIt, wait for completion

    async def close_gripper(self, distance):
        """Placeholder for gripper control"""
        print(f"Closing gripper to {distance:.3f}m")

    async def open_gripper(self):
        """Placeholder for gripper control"""
        print("Opening gripper")

# Usage
controller = PickAndPlaceController()

object_info = {
    'width': 0.05,
    'position': np.array([0.5, 0.2, 0.75])
}

place_location = np.array([0.3, -0.1, 0.80])

# await controller.execute_pick_and_place(object_info, place_location)
```

---

## Summary

- ✅ Grasp planning for parallel jaw grippers
- ✅ Force control for stable grasping
- ✅ Complete pick-and-place state machine
- ✅ Error handling and recovery

**Module 4 Chapter 13 Complete!**

Continue to [Chapter 14: Multi-Modal Integration →](../ch14-multimodal/vla-pipeline.md)
