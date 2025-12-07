# Advanced Domain Randomization

## Overview

Advanced domain randomization techniques go beyond basic parameter variation. This includes curriculum learning, adversarial randomization, and automatic domain randomization (ADR) that adapts randomization ranges during training for optimal sim-to-real transfer.

**What You'll Learn**:
- Curriculum learning for progressive difficulty
- Adversarial domain randomization
- Automatic Domain Randomization (ADR)
- Multi-modal randomization strategies

**Estimated Time**: 2 hours

---

## Part 1: Curriculum Learning

### Progressive Difficulty

**Concept**: Start easy, gradually increase challenge

```python
class CurriculumScheduler:
    def __init__(self):
        self.episode = 0
        self.difficulty = 0.0

    def update(self):
        self.episode += 1
        # Linear increase: 0.0 → 1.0 over 1000 episodes
        self.difficulty = min(1.0, self.episode / 1000.0)

    def get_randomization_range(self, base_range):
        # Scale randomization by difficulty
        return base_range * self.difficulty

# Usage
scheduler = CurriculumScheduler()

friction_range = scheduler.get_randomization_range([0.5, 1.5])
# Episode 0: friction in [1.0, 1.0] (no randomization)
# Episode 500: friction in [0.75, 1.25]
# Episode 1000+: friction in [0.5, 1.5]
```

---

## Part 2: Adversarial Randomization

### Maximize Policy Challenge

**Concept**: Sample parameters that cause policy to perform poorly

```python
def adversarial_randomization(policy, num_candidates=100):
    # Sample many randomizations
    candidates = []
    for _ in range(num_candidates):
        params = sample_random_params()
        reward = evaluate_policy(policy, params)
        candidates.append((params, reward))

    # Select worst-performing parameters
    worst_params = min(candidates, key=lambda x: x[1])[0]
    return worst_params
```

**Effect**: Policy trains on "hard" scenarios, becomes more robust

---

## Part 3: Automatic Domain Randomization (ADR)

### Algorithm

```python
class ADR:
    def __init__(self):
        self.param_ranges = {
            'friction': [0.9, 1.1],  # Start narrow
            'mass': [0.95, 1.05]
        }
        self.performance_threshold = 0.8  # 80% success rate

    def update_ranges(self, performance):
        if performance > self.performance_threshold:
            # Expand ranges (make harder)
            for param in self.param_ranges:
                lower, upper = self.param_ranges[param]
                self.param_ranges[param] = [lower * 0.95, upper * 1.05]
        else:
            # Shrink ranges (make easier)
            for param in self.param_ranges:
                lower, upper = self.param_ranges[param]
                self.param_ranges[param] = [lower * 1.02, upper * 0.98]

# Training loop
adr = ADR()
for episode in range(10000):
    params = sample_from_ranges(adr.param_ranges)
    reward = train_episode(params)
    adr.update_ranges(reward)
```

**Benefit**: Automatically finds optimal randomization difficulty

---

## Part 4: Multi-Modal Randomization

### Correlated Parameters

**Example**: Heavy robot → stronger motors

```python
def sample_correlated_params():
    mass_scale = random.uniform(0.8, 1.2)

    # Correlate motor strength with mass
    motor_scale = mass_scale * random.uniform(0.95, 1.05)

    # Correlate friction (heavier robot compresses ground)
    friction = 0.7 + 0.2 * (mass_scale - 1.0)

    return {
        'mass': mass_scale,
        'motor_strength': motor_scale,
        'friction': friction
    }
```

---

## Part 5: Validation

### Test on Holdout Randomizations

```python
# Training: Randomize in ranges [0.5, 1.5]
# Holdout: Test on [0.3, 0.5] and [1.5, 1.8]

holdout_performance = evaluate_on_holdout(policy)
if holdout_performance > 0.7:
    print("Policy generalizes well!")
```

---

## Summary

✅ Implemented curriculum learning for progressive training
✅ Applied adversarial randomization for robustness
✅ Used ADR for automatic difficulty adjustment
✅ Validated generalization on holdout sets

**Congratulations!** Module 3 (Isaac Sim + Perception + Navigation + RL) complete!

---

## Resources

- **ADR Paper**: https://arxiv.org/abs/1910.07113
- **Curriculum RL**: https://arxiv.org/abs/2003.04960
