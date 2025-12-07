#!/usr/bin/env python3
"""
Reinforcement Learning training script for humanoid locomotion
Uses Isaac Gym with PPO algorithm
"""

import torch
from isaacgym import gymapi
from rl_games.torch_runner import Runner
import yaml

def setup_environment():
    """Configure Isaac Gym environment"""
    env_config = {
        "name": "Humanoid",
        "physics_engine": gymapi.SIM_PHYSX,
        "num_envs": 4096,
        "env_spacing": 5.0,
        "episode_length": 1000,
        "enable_viewer_sync": True,
        "viewer": True,

        # Observation space (33D)
        "observations": {
            "base_lin_vel": 3,
            "base_ang_vel": 3,
            "projected_gravity": 3,
            "joint_pos": 12,
            "joint_vel": 12
        },

        # Action space (12D joint targets)
        "actions": {
            "type": "continuous",
            "dim": 12
        },

        # Reward weights
        "rewards": {
            "forward_velocity": 2.0,
            "alive_bonus": 1.0,
            "energy_penalty": 0.001,
            "orientation_penalty": 0.5
        }
    }

    return env_config

def setup_ppo():
    """Configure PPO algorithm"""
    ppo_config = {
        "params": {
            "seed": 42,
            "algo": {
                "name": "a2c_continuous"
            },
            "model": {
                "name": "continuous_a2c_logstd"
            },
            "network": {
                "name": "actor_critic",
                "separate": False,
                "space": {
                    "continuous": {
                        "mu_activation": "None",
                        "sigma_activation": "None",
                        "mu_init": {
                            "name": "default"
                        },
                        "sigma_init": {
                            "name": "const_initializer",
                            "val": 0
                        },
                        "fixed_sigma": True
                    }
                },
                "mlp": {
                    "units": [256, 128, 64],
                    "activation": "elu",
                    "d2rl": False,
                    "initializer": {
                        "name": "default"
                    },
                    "regularizer": {
                        "name": "None"
                    }
                }
            },
            "load_checkpoint": False,
            "config": {
                "name": "Humanoid_PPO",
                "full_experiment_name": "isaac_humanoid",
                "env_name": "rlgpu",
                "multi_gpu": False,
                "ppo": True,
                "mixed_precision": True,
                "normalize_input": True,
                "normalize_value": True,
                "num_actors": 4096,
                "reward_shaper": {
                    "scale_value": 1.0
                },
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
                "print_stats": True,
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

    return ppo_config

def main():
    print("=" * 60)
    print("Humanoid Locomotion Training with PPO")
    print("=" * 60)

    # Setup environment
    env_config = setup_environment()
    print(f"\nEnvironment: {env_config['num_envs']} parallel environments")
    print(f"Episode length: {env_config['episode_length']} steps")

    # Setup PPO
    ppo_config = setup_ppo()
    print(f"\nAlgorithm: PPO")
    print(f"Learning rate: {ppo_config['params']['config']['learning_rate']}")
    print(f"Max epochs: {ppo_config['params']['config']['max_epochs']}")

    # Initialize runner
    runner = Runner()
    runner.load(ppo_config)
    runner.reset()

    print("\nStarting training...")
    print("Monitor progress with: tensorboard --logdir=./runs/Humanoid_PPO")

    # Train
    runner.run({
        "train": True,
        "play": False,
        "checkpoint": None,
        "sigma": None
    })

    print("\n" + "=" * 60)
    print("Training Complete!")
    print("=" * 60)

if __name__ == "__main__":
    main()
