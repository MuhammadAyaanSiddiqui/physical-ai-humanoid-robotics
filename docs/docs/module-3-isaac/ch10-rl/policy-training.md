# Policy Training with Isaac Gym & RL

## Overview

**Reinforcement Learning (RL)** enables robots to learn complex behaviors through trial-and-error in simulation. Isaac Gym provides GPU-accelerated parallel environments where thousands of robots train simultaneously, learning policies (control strategies) for locomotion, manipulation, and navigation.

**What You'll Learn**:
- RL fundamentals (PPO, SAC algorithms)
- Train policies in Isaac Gym for humanoid locomotion
- Configure reward functions for desired behaviors
- Monitor training progress with TensorBoard
- Export trained policies for deployment

**Prerequisites**:
- Python and PyTorch basics
- Understanding of neural networks
- Completed Isaac Sim chapters

**Estimated Time**: 4 hours

---

## Learning Objectives

1. Understand RL concepts (states, actions, rewards, policies)
2. Set up Isaac Gym training environment
3. Train locomotion policy with PPO
4. Tune hyperparameters for stable training
5. Deploy trained policy in Isaac Sim

---

## Part 1: RL Fundamentals

### Key Concepts

| Concept | Description | Example |
|---------|-------------|---------|
| **State** | Robot's observation of environment | Joint angles, velocities, IMU |
| **Action** | Robot's control output | Joint torques |
| **Reward** | Feedback signal (+good, -bad) | +1 for forward progress, -10 for falling |
| **Policy** | Strategy mapping states → actions | Neural network |
| **Value Function** | Expected cumulative reward from state | Estimate total future reward |

### PPO (Proximal Policy Optimization)

**Best for**: Robotics (stable, sample-efficient)

**Algorithm**:
1. Collect trajectories (robot episodes)
2. Compute advantages (how good actions were)
3. Update policy via gradient ascent
4. Clip updates to prevent large changes (stability)

---

## Part 2: Isaac Gym Setup

### Installation

```bash
# Download Isaac Gym (NVIDIA Developer account required)
wget https://developer.nvidia.com/isaac-gym

# Extract and install
cd isaacgym/python
pip install -e .
```

### Verify Installation

```python
import isaacgym
from isaacgym import gymapi

gym = gymapi.acquire_gym()
print("Isaac Gym initialized successfully!")
```

---

## Part 3: Training Humanoid Locomotion

### Task: Walk Forward

**Goal**: Train humanoid to walk forward without falling

**State** (observations):
- Base linear/angular velocity (6D)
- Joint positions/velocities (24D)
- Gravity vector (3D)
**Total**: 33D observation space

**Actions**:
- Target joint positions (12D)

**Reward Function**:

```python
def compute_reward(self):
    # Positive rewards
    forward_velocity_reward = self.base_lin_vel[0]  # Reward forward motion
    alive_reward = 1.0  # Reward staying upright

    # Negative penalties
    energy_penalty = -0.01 * torch.sum(torch.square(self.torques))
    orientation_penalty = -torch.sum(torch.square(self.projected_gravity[:, :2]))

    total_reward = forward_velocity_reward + alive_reward + energy_penalty + orientation_penalty
    return total_reward
```

### Training Script

```python
from rl_games.torch_runner import Runner
from isaacgymenvs.tasks import isaacgym_task_map

# Load humanoid environment
task = isaacgym_task_map["Humanoid"]()

# Configure PPO
config = {
    "params": {
        "algo": {
            "name": "a2c_continuous"
        },
        "network": {
            "name": "actor_critic",
            "separate": False,
            "space": {
                "continuous": {
                    "mu_activation": "None",
                    "sigma_activation": "None",
                    "mu_init": {"name": "default"},
                    "sigma_init": {"name": "const_initializer", "val": 0},
                    "fixed_sigma": True
                }
            },
            "mlp": {
                "units": [256, 128, 64],
                "activation": "elu",
                "d2rl": False,
                "initializer": {"name": "default"}
            }
        },
        "config": {
            "name": "Humanoid",
            "env_name": "rlgpu",
            "multi_gpu": False,
            "ppo": True,
            "mixed_precision": True,
            "normalize_input": True,
            "normalize_value": True,
            "value_bootstrap": True,
            "num_actors": 4096,  # Parallel environments
            "reward_shaper": {},
            "normalize_advantage": True,
            "gamma": 0.99,
            "tau": 0.95,
            "learning_rate": 3e-4,
            "lr_schedule": "adaptive",
            "kl_threshold": 0.008,
            "score_to_win": 20000,
            "max_epochs": 2000,
            "save_best_after": 100,
            "save_frequency": 50,
            "grad_norm": 1.0,
            "entropy_coef": 0.0,
            "truncate_grads": True,
            "e_clip": 0.2,
            "horizon_length": 16,
            "minibatch_size": 32768,
            "mini_epochs": 5,
            "critic_coef": 4,
            "clip_value": True,
            "seq_len": 4,
            "bounds_loss_coef": 0.0001
        }
    }
}

# Train
runner = Runner()
runner.load(config)
runner.reset()
runner.run({})
```

### Training Monitoring

```bash
# Launch TensorBoard
tensorboard --logdir=./runs/Humanoid
```

**Metrics to watch**:
- `ep_len_mean`: Episode length (longer = better)
- `ep_rew_mean`: Average reward (should increase)
- `approxkl`: KL divergence (should stay &lt;0.015 for stability)

**Training Time**: 1-2 hours on RTX 4070 Ti (2000 epochs, 4096 parallel envs)

---

## Part 4: Hyperparameter Tuning

### Key Parameters

| Parameter | Effect | Tuning Tips |
|-----------|--------|-------------|
| `learning_rate` | Update step size | Start 3e-4, reduce if unstable |
| `num_actors` | Parallel environments | More = faster, needs more GPU memory |
| `horizon_length` | Trajectory length | 8-32 typical for locomotion |
| `gamma` | Discount factor | 0.99 standard (values future rewards) |
| `entropy_coef` | Exploration bonus | 0.0-0.01 (higher = more exploration) |

### Common Issues

**Issue**: Policy collapses (reward drops suddenly)
**Solution**: Reduce `learning_rate` to 1e-4, increase `kl_threshold`

**Issue**: Training plateaus
**Solution**: Increase `entropy_coef` for more exploration

---

## Part 5: Deploying Trained Policy

### Export Policy

```python
# Save checkpoint
torch.save({
    'model_state_dict': agent.model.state_dict(),
    'optimizer_state_dict': agent.optimizer.state_dict(),
}, 'humanoid_policy.pth')
```

### Load and Run Policy

```python
import torch

# Load trained model
policy = ActorCritic(obs_dim=33, action_dim=12)
policy.load_state_dict(torch.load('humanoid_policy.pth')['model_state_dict'])
policy.eval()

# Run in Isaac Sim
with torch.no_grad():
    action = policy(observation)
    robot.apply_action(action)
```

---

## Summary

✅ Trained humanoid locomotion with PPO in Isaac Gym
✅ Configured reward functions for walking behavior
✅ Monitored training with TensorBoard
✅ Deployed trained policy

**Next**: Reward function design for custom tasks.

---

## Resources

- **Isaac Gym**: https://developer.nvidia.com/isaac-gym
- **RL Games**: https://github.com/Denys88/rl_games
- **PPO Paper**: https://arxiv.org/abs/1707.06347
