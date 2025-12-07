# Module 3 Assessment: Perception + Navigation Project

## Overview

This assessment evaluates your mastery of Isaac Sim, perception systems, autonomous navigation, and reinforcement learning. You will build a complete autonomous robot system that integrates all Module 3 skills.

**Estimated Time**: 8-12 hours
**Weight**: 25% of final course grade

---

## Project Requirements

### Task: Autonomous Warehouse Navigation & Object Detection

Build a system where a robot:
1. Maps a warehouse environment using VSLAM
2. Detects and localizes objects (boxes, pallets) using YOLO
3. Autonomously navigates to detected objects
4. Avoids dynamic obstacles
5. Reports completion via ROS 2 action server

---

## Part 1: Environment Setup (10 points)

**Requirements**:
- [ ] Load Isaac Sim warehouse environment
- [ ] Add Carter robot with stereo camera and LiDAR
- [ ] Enable ROS 2 bridge for all sensor topics
- [ ] Verify sensor data publishing at correct rates

**Deliverable**: Screenshot of Isaac Sim with robot and RViz showing sensor data

---

## Part 2: VSLAM Mapping (20 points)

**Requirements**:
- [ ] Configure Isaac ROS Visual SLAM
- [ ] Drive robot to explore environment
- [ ] Build complete map (>90% coverage)
- [ ] Detect loop closures (minimum 3)
- [ ] Save map in PGM + YAML format

**Metrics**:
- Map coverage: >90% = Full credit, 80-90% = Partial credit
- Loop closure drift: &lt;0.5m = Full credit, 0.5-1.0m = Partial credit

**Deliverable**: Saved map files + RViz screenshot showing complete map and trajectory

---

## Part 3: Object Detection (20 points)

**Requirements**:
- [ ] Train custom YOLOv8 model on warehouse objects (boxes, pallets)
- [ ] Achieve mAP > 0.75 on test set
- [ ] Run real-time detection (>20 FPS)
- [ ] Publish detections to `/detections` topic

**Metrics**:
- Detection mAP: >0.80 = Full credit, 0.75-0.80 = Partial credit
- Inference FPS: >30 FPS = Full, 20-30 FPS = Partial

**Deliverable**: Trained model checkpoint + detection visualization video

---

## Part 4: Autonomous Navigation (30 points)

**Requirements**:
- [ ] Configure Nav2 with tuned parameters
- [ ] Implement behavior tree with recovery behaviors
- [ ] Navigate to 5 waypoints successfully
- [ ] Avoid 3 dynamic obstacles during navigation
- [ ] Complete mission within time limit

**Metrics**:
- Waypoints reached: 5/5 = Full, 4/5 = Partial, &lt;4 = Fail
- Obstacle collisions: 0 = Full, 1 = Partial, >1 = Fail
- Time limit: &lt;5 minutes

**Deliverable**: Video recording of complete navigation run + Nav2 configuration files

---

## Part 5: Integration (20 points)

**Requirements**:
- [ ] Detect object with YOLO
- [ ] Get 3D position using depth estimation
- [ ] Generate navigation goal from object position
- [ ] Navigate to object (&lt;0.5m final distance)
- [ ] Repeat for 3 different objects

**Metrics**:
- Detection accuracy: All 3 objects detected = Full
- Navigation accuracy: Final distance &lt;0.5m for all = Full
- Integration robustness: No manual intervention = Full

**Deliverable**: Python script + demonstration video

---

## Bonus Challenges (Extra Credit: +20 points)

**Choose ONE**:

### Option A: RL-Based Navigation (+20 points)
- Train PPO policy for obstacle avoidance in Isaac Gym
- Deploy policy in Nav2 as custom planner plugin
- Demonstrate smoother navigation than DWA

**Deliverable**: Training logs, deployed policy, comparison video

### Option B: Multi-Robot Coordination (+20 points)
- Add 2nd robot to environment
- Implement coordination to avoid inter-robot collisions
- Complete task with both robots simultaneously

**Deliverable**: Coordination logic code, demonstration video

### Option C: Real-World Deployment (+20 points)
- Deploy VSLAM + detection on physical robot (if available)
- Measure sim-to-real performance gap
- Document tuning required for real hardware

**Deliverable**: Real robot video, performance comparison report

---

## Submission Guidelines

### Required Files

```
assessment-submission/
├── README.md                   # Project overview, setup instructions
├── maps/
│   ├── warehouse_map.pgm
│   └── warehouse_map.yaml
├── models/
│   └── yolov8_warehouse.pt
├── config/
│   ├── nav2_params.yaml
│   └── behavior_tree.xml
├── scripts/
│   ├── object_navigation.py
│   └── assessment_demo.py
├── videos/
│   ├── vslam_mapping.mp4
│   ├── object_detection.mp4
│   ├── autonomous_navigation.mp4
│   └── full_integration.mp4
└── report.pdf                  # Written report (see below)
```

### Written Report (5 pages max)

**Sections**:
1. **Introduction** (0.5 page): Task overview and approach
2. **System Architecture** (1 page): Component diagram and data flow
3. **Implementation Details** (2 pages):
   - VSLAM configuration and tuning
   - Object detection training process
   - Nav2 parameter tuning
   - Integration challenges and solutions
4. **Results** (1 page):
   - Performance metrics table
   - Comparison to requirements
   - Analysis of failures (if any)
5. **Conclusion** (0.5 page): Lessons learned, future improvements

---

## Grading Rubric

| Component | Points | Criteria |
|-----------|--------|----------|
| **Environment Setup** | 10 | Correct scene, sensors, ROS 2 bridge |
| **VSLAM Mapping** | 20 | Map quality, coverage, loop closures |
| **Object Detection** | 20 | Model accuracy, FPS, real-time performance |
| **Autonomous Navigation** | 30 | Waypoint success, collision avoidance, time |
| **Integration** | 20 | End-to-end functionality, robustness |
| **Bonus (Optional)** | +20 | Extra credit challenges |
| **Total** | 100 | (120 with bonus) |

### Grading Scale

- **A (90-100)**: Exceeds all requirements, high-quality code, excellent report
- **B (80-89)**: Meets all requirements, good code quality, solid report
- **C (70-79)**: Meets most requirements, functional code, adequate report
- **D (60-69)**: Meets minimum requirements, basic functionality
- **F (&lt;60)**: Fails to meet minimum requirements

---

## Tips for Success

1. **Start early**: This is a comprehensive project requiring 8-12 hours
2. **Test incrementally**: Validate each part before integration
3. **Use version control**: Git commit after each working milestone
4. **Document as you go**: Don't leave report writing to the end
5. **Ask for help**: Use course forums, office hours
6. **Record everything**: Capture videos during development for submission

---

## Common Pitfalls to Avoid

- **VSLAM failure**: Ensure good lighting and texture in environment
- **Detection gaps**: Train on diverse synthetic data with domain randomization
- **Navigation stalls**: Tune inflation radius and recovery behaviors
- **Integration timing**: Synchronize detection and depth data properly
- **Performance issues**: Monitor CPU/GPU usage, optimize sensor rates

---

## Submission Deadline

**Due**: End of Week 10 (Module 3 completion)

**Late Policy**: -10% per day, max 3 days late

---

## Academic Integrity

- **Allowed**: Course materials, official documentation, peer discussion
- **Not Allowed**: Copying code from others, sharing solutions, using external complete implementations

Violations will result in a grade of 0 and referral to academic conduct board.

---

## Questions?

Contact instructors via:
- Course forum: https://course.example.com/forum
- Office hours: Tuesdays 2-4 PM
- Email: instructors@example.com

---

**Good luck! This project showcases the culmination of your Physical AI perception and navigation skills.**
