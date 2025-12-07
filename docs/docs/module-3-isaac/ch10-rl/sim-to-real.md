# Sim-to-Real Transfer

## Overview

**Sim-to-real transfer** deploys policies trained in simulation on physical robots. Success requires domain randomization (vary simulation parameters) to build robust policies that generalize despite simulation imperfections.

**What You'll Learn**:
- Sim-to-real gap causes and solutions
- Domain randomization for robust policies
- System identification for accurate simulation
- Validation metrics for transfer success

**Estimated Time**: 2 hours

---

## Part 1: The Sim-to-Real Gap

### Why Policies Fail on Real Robots

| Aspect | Simulation | Reality | Gap |
|--------|------------|---------|-----|
| **Physics** | Perfect friction/damping | Varies with surface/temperature | Medium |
| **Latency** | Instant sensor/actuator | 5-50ms delays | Large |
| **Noise** | Optional Gaussian | Non-Gaussian, biases | Medium |
| **Modeling** | Simplified kinematics | Manufacturing tolerances | Small |

---

## Part 2: Domain Randomization

### Randomize Physics Parameters

```python
class HumanoidEnv:
    def randomize_domain(self):
        # Randomize friction
        self.friction = torch.rand(self.num_envs) * 0.5 + 0.5  # [0.5, 1.0]

        # Randomize mass
        self.mass_scale = torch.rand(self.num_envs) * 0.4 + 0.8  # [0.8, 1.2]

        # Randomize motor strength
        self.torque_scale = torch.rand(self.num_envs) * 0.3 + 0.85  # [0.85, 1.15]

        # Apply to simulation
        for i in range(self.num_envs):
            gym.set_actor_dof_properties(env, actor, dof_props * self.torque_scale[i])
```

### Randomize Observations (Sensor Noise)

```python
def add_sensor_noise(self, obs):
    # IMU noise
    obs[:, :3] += torch.randn_like(obs[:, :3]) * 0.01  # Gyro noise

    # Joint encoder noise
    obs[:, 6:18] += torch.randn_like(obs[:, 6:18]) * 0.002  # Position noise

    return obs
```

---

## Part 3: System Identification

### Measure Real Robot Parameters

```python
# Measure friction coefficient
def measure_friction():
    # Apply constant force, measure slip velocity
    force = 10.0  # Newtons
    velocity = measure_velocity()  # m/s

    friction = force / (mass * velocity)  # Simplified
    return friction

measured_friction = measure_friction()
print(f"Real robot friction: {measured_friction}")
```

### Update Simulation

```yaml
# Isaac Sim physics config
humanoid:
  friction: 0.73  # Measured value
  restitution: 0.05
  mass_scale: 1.02  # Account for wiring
```

---

## Part 4: Validation Metrics

### Transfer Success Metrics

| Metric | Simulation | Real Robot | Transfer Gap |
|--------|------------|------------|--------------|
| **Forward velocity** | 1.2 m/s | 0.95 m/s | 21% |
| **Energy efficiency** | 50 J/m | 68 J/m | 36% |
| **Stability** | 98% uptime | 91% uptime | 7% |

**Target**: &lt;20% gap for deployment

---

## Part 5: Best Practices

1. **Start conservative**: Narrow randomization ranges, expand gradually
2. **Validate iteratively**: Test small changes on real robot frequently
3. **Prioritize safety**: Add torque limits, emergency stops
4. **Log everything**: Record all real-world trials for debugging

---

## Summary

✅ Understood sim-to-real gap
✅ Applied domain randomization for robustness
✅ Measured real robot parameters
✅ Validated transfer success

**Next**: Advanced domain randomization techniques.

---

## Resources

- **Sim-to-Real Survey**: https://arxiv.org/abs/2109.09928
