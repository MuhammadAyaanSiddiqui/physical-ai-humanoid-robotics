---
sidebar_position: 4
---

# Course Roadmap

## Course Structure

This 13-week course is designed to take you from Physical AI fundamentals to building an autonomous humanoid robot with voice control. The curriculum follows a progressive, hands-on approach where each module builds upon the previous one.

## Weekly Breakdown

### Weeks 1-2: Foundations
**Goal**: Understand Physical AI concepts and sensor ecosystems

**Topics**:
- What is Physical AI?
- Embodied intelligence
- Sensor modalities (cameras, LiDAR, IMU)
- Hardware overview

**Deliverables**:
- Conceptual quiz on Physical AI
- Sensor comparison analysis

---

### Weeks 3-5: ROS 2 Robot Control Mastery
**Goal**: Master ROS 2 fundamentals for robot programming

**Topics**:
- ROS 2 installation and setup (Ubuntu 22.04 + Humble)
- Nodes, topics, services, and actions
- Robot modeling with URDF
- Creating ROS 2 packages
- Launch files and parameters

**Hands-On Projects**:
- Create a publisher-subscriber system
- Build a service-based calculator
- Model a custom robot in URDF
- Publish joint states and visualize in RViz

**Deliverables**:
- Functional ROS 2 package with custom nodes
- URDF robot model with articulated joints

---

### Weeks 6-7: Digital Twin Simulation Environments
**Goal**: Build simulation environments for robot testing

**Topics**:
- Gazebo Fortress fundamentals
- Unity for robotics simulation
- Sensor modeling (camera, LiDAR)
- Physics engines and contact dynamics
- World building and asset creation

**Hands-On Projects**:
- Spawn a robot in Gazebo with sensors
- Create a custom warehouse environment
- Simulate LiDAR and camera data
- Test teleoperation in simulation

**Deliverables**:
- Custom Gazebo world with obstacles
- Simulated robot with functional sensors

---

### Weeks 8-10: NVIDIA Isaac for AI-Powered Perception
**Goal**: Add perception and navigation capabilities using Isaac

**Topics**:
- Isaac Sim overview and setup
- Visual SLAM (VSLAM) with Isaac ROS
- Object detection with DOPE
- Path planning with Nav2
- Sim-to-real transfer techniques

**Hands-On Projects**:
- Run VSLAM in Isaac Sim
- Detect and track objects with DOPE
- Navigate autonomously using Nav2
- Transfer a trained policy to Jetson Orin

**Deliverables**:
- VSLAM map of a simulated environment
- Autonomous navigation demo
- Object detection pipeline

---

### Weeks 11-12: Vision-Language-Action Cognitive Robotics
**Goal**: Build voice-controlled cognitive robots

**Topics**:
- Speech recognition with Whisper
- LLM-based task planning (GPT-4, LangChain)
- Grounding language to robot actions
- Conversational AI for robotics

**Hands-On Projects**:
- Transcribe voice commands with Whisper
- Generate robot action sequences from natural language
- Integrate LLM with ROS 2 action server
- Build a conversational robot interface

**Deliverables**:
- Voice-to-action pipeline
- LLM-based task planner

---

### Week 13: Capstone - Autonomous Humanoid System
**Goal**: Integrate all modules into an end-to-end humanoid robot

**Capstone Requirements**:
1. **Voice Input**: User speaks a command
2. **Speech Recognition**: Whisper transcribes audio
3. **Task Planning**: LLM generates action plan
4. **Navigation**: Robot navigates to target using VSLAM
5. **Perception**: Detect and localize objects with Isaac
6. **Manipulation**: Pick/place objects (simulated or real)

**Example Scenario**:
```
User: "Go to the kitchen and bring me a water bottle"
Robot:
  1. Transcribes command with Whisper
  2. LLM plans: [navigate_to(kitchen), detect(water_bottle), grasp(water_bottle), navigate_to(user), release(water_bottle)]
  3. Executes navigation using VSLAM + Nav2
  4. Detects water bottle with Isaac perception
  5. Grasps bottle (simulated gripper)
  6. Returns to user
```

**Deliverables**:
- End-to-end demo video
- System architecture diagram
- Capstone report (3-5 pages)

---

## Learning Outcomes by Module

| Module | Learning Outcomes |
|--------|------------------|
| **Foundations** | Define Physical AI, identify sensor types, explain embodied intelligence |
| **ROS 2** | Create ROS 2 nodes, model robots in URDF, build packages, use launch files |
| **Simulation** | Build Gazebo worlds, simulate sensors, test robots in simulation |
| **Isaac AI** | Run VSLAM, detect objects, navigate autonomously, deploy to edge devices |
| **VLA** | Transcribe speech, plan with LLMs, ground language to actions |
| **Capstone** | Integrate multi-modal AI, build end-to-end systems, evaluate performance |

## Assessments

### Weekly Quizzes (10%)
- Conceptual questions on key topics
- Multiple choice + short answer

### Coding Assignments (40%)
- ROS 2 package development (Week 5)
- Simulation project (Week 7)
- Isaac perception pipeline (Week 10)
- VLA integration (Week 12)

### Capstone Project (50%)
- System design (10%)
- Implementation (30%)
- Demonstration (10%)

## Hardware Requirements

### Option 1: Simulation-Only (Budget: $0-$500)
- **Workstation**: Desktop/laptop with GPU (GTX 1060+)
- **Software**: ROS 2, Gazebo, Isaac Sim (cloud)

### Option 2: Cloud-Based Development (Budget: $500-$1,000)
- **Workstation**: Standard laptop
- **Cloud**: AWS RoboMaker or NVIDIA Omniverse Cloud
- **Edge Device**: NVIDIA Jetson Orin Nano ($500)

### Option 3: Full Hardware Setup (Budget: $3,000-$8,000)
- **Workstation**: RTX 4070+ GPU, 32GB RAM
- **Edge Kit**: Jetson Orin Nano + peripherals ($500)
- **Robot**: Unitree Go2 Edu ($3,000) or G1 Humanoid ($16,000)

## Prerequisites

### Required
- Basic programming (Python preferred)
- Linux familiarity (basic terminal commands)
- High school mathematics (algebra, trigonometry)

### Recommended
- Computer vision basics
- Control theory (PID controllers)
- Machine learning fundamentals

## Time Commitment

- **Lectures/Reading**: 3-4 hours/week
- **Hands-On Projects**: 5-7 hours/week
- **Capstone**: 15-20 hours total

**Total**: ~100-120 hours over 13 weeks

## Next Steps

Ready to begin? Start with **Module 1: ROS 2 Robot Control Mastery** by setting up your development environment.

[Go to ROS 2 Installation →](../module-1-ros2/ch1-fundamentals/installation.md)

---

**Prerequisites**: None (this is an overview)
**Estimated Time**: 10 minutes
**Learning Outcomes**: Understand course structure, identify learning outcomes, assess hardware requirements
