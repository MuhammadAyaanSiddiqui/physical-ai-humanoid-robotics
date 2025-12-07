# Reward Function Design

## Overview

**Reward engineering** is critical for RL success. A well-designed reward function guides the policy to desired behaviors while avoiding unintended shortcuts. This lesson covers principles, common pitfalls, and techniques for crafting effective reward functions.

**What You'll Learn**:
- Reward shaping principles
- Avoid reward hacking and degenerate solutions
- Design multi-objective rewards
- Debug reward functions with logging

**Estimated Time**: 2 hours

---

## Part 1: Reward Shaping Principles

### Sparse vs. Dense Rewards

**Sparse**: Reward only at goal (e.g., +100 for reaching target, 0 otherwise)
- **Pros**: Simple, well-defined
- **Cons**: Hard to learn (rare positive signal)

**Dense**: Reward every step (e.g., -distance_to_goal)
- **Pros**: Faster learning
- **Cons**: Risk of reward hacking

### Multi-Component Rewards

```python
def compute_reward(self):
    # Primary objective
    forward_reward = 2.0 * self.base_lin_vel[0]

    # Secondary objectives
    upright_reward = 1.0 * (self.projected_gravity[2] > 0.9)
    energy_penalty = -0.001 * torch.sum(torch.square(self.torques))
    alive_bonus = 0.5

    total = forward_reward + upright_reward + energy_penalty + alive_bonus
    return total
```

---

## Part 2: Common Reward Hacking

### Issue 1: Spinning in Place

**Bad Reward**: `reward = angular_velocity`
**Hack**: Robot spins rapidly (high reward, no forward progress)
**Fix**: `reward = forward_velocity - 0.1 * abs(angular_velocity)`

### Issue 2: Tip-Toeing

**Bad Reward**: `reward = height`
**Hack**: Robot balances on toes (high center of mass)
**Fix**: `reward = height * (contact_area > threshold)`

---

## Part 3: Reward Components

### Distance-Based

```python
distance_to_goal = torch.norm(self.root_pos[:, :2] - goal_pos, dim=1)
reward = -distance_to_goal  # Minimize distance
```

### Velocity-Based

```python
desired_vel = 1.0  # m/s
actual_vel = self.base_lin_vel[0]
reward = -abs(desired_vel - actual_vel)  # Match desired speed
```

### Orientation-Based

```python
# Penalize tilting
gravity_z = self.projected_gravity[:, 2]  # Should be ~1.0 when upright
reward = gravity_z  # Reward staying vertical
```

### Contact-Based

```python
# Penalize excessive foot slippage
foot_slip = torch.norm(self.foot_velocities_xy, dim=1)
reward = -0.1 * foot_slip
```

---

## Part 4: Debugging Rewards

### Logging Reward Components

```python
self.extras['forward_reward'] = forward_reward.mean()
self.extras['energy_penalty'] = energy_penalty.mean()
self.extras['alive_bonus'] = alive_bonus.mean()

# View in TensorBoard
```

### Visualization

```python
# Plot reward over episode
import matplotlib.pyplot as plt

plt.plot(episode_rewards)
plt.xlabel('Timestep')
plt.ylabel('Reward')
plt.title('Reward Progress')
plt.show()
```

---

## Summary

✅ Designed multi-component reward functions
✅ Avoided reward hacking with constraints
✅ Debugged rewards with logging

**Next**: Sim-to-real transfer strategies.

---

## Resources

- **Reward Engineering**: https://spinningup.openai.com/en/latest/spinningup/rl_intro3.html
