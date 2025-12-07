# Bipedal Locomotion and Walking Gaits

## Learning Objectives

- Understand bipedal walking principles (ZMP, COM, gait cycles)
- Implement walking gaits for humanoid robots
- Use Zero Moment Point (ZMP) for balance control
- Generate footstep plans and trajectories
- Simulate walking in Isaac Sim

**Prerequisites**: Kinematics, Python, ROS 2, Isaac Sim

**Estimated Time**: 4 hours

---

## Introduction

Bipedal walking involves:
- **Gait cycle**: Swing phase (foot in air) + Stance phase (foot on ground)
- **ZMP (Zero Moment Point)**: Point where net moment = 0 (must stay in support polygon)
- **COM (Center of Mass)**: Must be controlled to maintain balance

---

## Part 1: Gait Fundamentals

### Walking Cycle Phases

```python
class GaitPhase:
    DOUBLE_SUPPORT = "double_support"  # Both feet on ground
    LEFT_SWING = "left_swing"          # Right foot support
    RIGHT_SWING = "right_swing"        # Left foot support

    def __init__(self):
        self.current_phase = self.DOUBLE_SUPPORT
        self.phase_duration = 0.1  # seconds
```

---

## Part 2: ZMP Control

```python
import numpy as np

def compute_zmp(com_position, com_acceleration, gravity=9.81):
    """
    Calculate ZMP from COM state

    ZMP_x = COM_x - (COM_z / g) * COM_acc_x
    """
    zmp_x = com_position[0] - (com_position[2] / gravity) * com_acceleration[0]
    zmp_y = com_position[1] - (com_position[2] / gravity) * com_acceleration[1]

    return np.array([zmp_x, zmp_y])

# Example
com_pos = np.array([0.0, 0.0, 0.9])  # meters
com_acc = np.array([0.5, 0.0, 0.0])   # m/s^2

zmp = compute_zmp(com_pos, com_acc)
print(f"ZMP: x={zmp[0]:.3f}, y={zmp[1]:.3f}")
```

---

## Part 3: Footstep Planning

```python
class FootstepPlanner:
    def __init__(self, step_length=0.2, step_width=0.15):
        self.step_length = step_length  # Forward step distance
        self.step_width = step_width    # Lateral foot spacing

    def generate_forward_steps(self, num_steps):
        """Generate footsteps for forward walking"""
        footsteps = []

        for i in range(num_steps):
            if i % 2 == 0:  # Left foot
                x = (i // 2) * self.step_length
                y = self.step_width / 2
                foot = "left"
            else:  # Right foot
                x = (i // 2 + 1) * self.step_length
                y = -self.step_width / 2
                foot = "right"

            footsteps.append({
                'foot': foot,
                'position': np.array([x, y, 0.0]),
                'orientation': 0.0  # yaw angle
            })

        return footsteps

# Generate 10 steps
planner = FootstepPlanner(step_length=0.25, step_width=0.20)
steps = planner.generate_forward_steps(10)

for i, step in enumerate(steps):
    print(f"Step {i}: {step['foot']} at ({step['position'][0]:.2f}, {step['position'][1]:.2f})")
```

---

## Summary

- ✅ Gait cycles and phases
- ✅ ZMP computation for balance
- ✅ Footstep planning algorithms

Continue to [Balance Control →](./balance-control.md)
