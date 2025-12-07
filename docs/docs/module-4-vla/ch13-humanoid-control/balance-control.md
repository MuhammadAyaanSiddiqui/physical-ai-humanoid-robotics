# Balance Control and Stabilization

## Learning Objectives

- Implement COM (Center of Mass) tracking
- Use PID controllers for balance stabilization
- Handle external disturbances (pushes, uneven terrain)
- Integrate balance control with walking gaits

**Estimated Time**: 3 hours

---

## Part 1: COM Tracking

```python
import numpy as np

class COMController:
    def __init__(self, kp=50.0, kd=10.0):
        self.kp = kp  # Proportional gain
        self.kd = kd  # Derivative gain
        self.com_error = np.zeros(3)
        self.com_error_dot = np.zeros(3)

    def compute_force(self, com_desired, com_actual, com_velocity):
        """
        Compute corrective force to track desired COM

        F = Kp * (com_desired - com_actual) + Kd * (0 - com_velocity)
        """
        self.com_error = com_desired - com_actual
        self.com_error_dot = -com_velocity

        force = self.kp * self.com_error + self.kd * self.com_error_dot

        return force

# Example
controller = COMController(kp=50.0, kd=10.0)

com_desired = np.array([0.0, 0.0, 0.9])
com_actual = np.array([0.02, -0.01, 0.88])
com_vel = np.array([0.1, 0.05, -0.02])

force = controller.compute_force(com_desired, com_actual, com_vel)
print(f"Corrective force: {force}")
```

---

## Part 2: Disturbance Rejection

```python
class BalanceRecovery:
    def __init__(self):
        self.max_recovery_step = 0.3  # meters

    def compute_recovery_step(self, com_position, com_velocity, zmp_position):
        """
        Compute recovery step when pushed

        Uses Capture Point method
        """
        omega = np.sqrt(9.81 / com_position[2])  # Natural frequency

        # Capture point = where COM will be if we don't act
        capture_point = com_position[:2] + com_velocity[:2] / omega

        # Recovery step should land at capture point
        recovery_step = capture_point - zmp_position

        # Limit step size
        step_magnitude = np.linalg.norm(recovery_step)
        if step_magnitude > self.max_recovery_step:
            recovery_step = recovery_step / step_magnitude * self.max_recovery_step

        return recovery_step

recovery = BalanceRecovery()

# Robot pushed forward
com_pos = np.array([0.0, 0.0, 0.9])
com_vel = np.array([0.8, 0.0, 0.0])  # Fast forward velocity!
zmp = np.array([0.0, 0.0])

step = recovery.compute_recovery_step(com_pos, com_vel, zmp)
print(f"Recovery step: {step} meters")
```

---

## Summary

- ✅ COM tracking with PID control
- ✅ Disturbance rejection strategies
- ✅ Capture Point method for recovery

Continue to [Manipulation →](./manipulation.md)
