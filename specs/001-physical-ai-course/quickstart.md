# Quickstart Guide: Physical AI & Humanoid Robotics Course

**Welcome!** This guide will help you get started with the Physical AI & Humanoid Robotics course and complete your first hands-on exercise.

## Prerequisites

Before starting, ensure you have:

- **Programming**: Basic Python knowledge (variables, functions, loops, classes)
- **Command Line**: Familiarity with Linux bash (cd, ls, mkdir, chmod)
- **Math**: Basic understanding of 2D/3D coordinates
- **Time**: 8-12 hours per week for 13 weeks
- **Hardware**: Computer capable of running Ubuntu 22.04 (see Hardware Decision Tree below)

## Environment Setup (Essential)

### Step 1: Install Ubuntu 22.04 LTS

**Option A: Native/Dual-Boot (Recommended)**
1. Download Ubuntu 22.04 LTS from [ubuntu.com](https://ubuntu.com/download/desktop)
2. Create bootable USB with [Rufus](https://rufus.ie/) (Windows) or [Etcher](https://etcher.balena.io/) (Mac/Linux)
3. Install Ubuntu alongside existing OS or as sole OS

**Option B: WSL2 (Windows only, limited for Isaac Sim)**
1. Enable WSL2: `wsl --install`
2. Install Ubuntu 22.04 from Microsoft Store
3. Note: WSL2 works for ROS 2 and Gazebo, but NOT recommended for Isaac Sim

**Option C: Virtual Machine (Not recommended - performance issues)**

### Step 2: Install ROS 2 Humble

Open terminal and run:

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop

# Install development tools
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

### Step 3: Source ROS 2 (add to ~/.bashrc for persistence)

```bash
source /opt/ros/humble/setup.bash
```

**Make it permanent:**
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 4: Verify Installation

```bash
# Check ROS 2 version
ros2 --version
# Expected: ros2 cli version 0.18.x

# List available ROS 2 packages
ros2 pkg list | head -n 5

# Run demo nodes
ros2 run demo_nodes_cpp talker
# In another terminal: ros2 run demo_nodes_py listener
```

If you see messages being published and received, congratulations! ROS 2 is working.

## First Hands-On Exercise: Hello ROS 2

### Create Your First Publisher/Subscriber

**Step 1: Create a workspace**

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

**Step 2: Create publisher node**

Create file `hello_publisher.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class HelloPublisher(Node):
    def __init__(self):
        super().__init__('hello_publisher')
        self.publisher = self.create_publisher(String, 'hello_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.count = 0

    def publish_message(self):
        msg = String()
        msg.data = f'Hello Physical AI: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main():
    rclpy.init()
    node = HelloPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Step 3: Make executable and run**

```bash
chmod +x hello_publisher.py
python3 hello_publisher.py
```

**Expected Output:**
```
[INFO] [hello_publisher]: Publishing: "Hello Physical AI: 0"
[INFO] [hello_publisher]: Publishing: "Hello Physical AI: 1"
[INFO] [hello_publisher]: Publishing: "Hello Physical AI: 2"
...
```

**Step 4: Listen to the topic (in another terminal)**

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /hello_topic
```

**Troubleshooting:**
- "ModuleNotFoundError: No module named 'rclpy'" → Source ROS 2: `source /opt/ros/humble/setup.bash`
- No output → Check node is running: `ros2 node list`
- Topic not found → Verify topic exists: `ros2 topic list`

## Navigation Guide

### How to Use This Book

**Progressive Path (Recommended for Beginners):**
Follow modules sequentially: Intro → Module 1 (ROS 2) → Module 2 (Simulation) → Module 3 (Isaac) → Module 4 (VLA) → Capstone

**Targeted Path (For Experienced Learners):**
Jump to specific modules based on your interests, but review prerequisites for each module.

### Module Overview

| Module | Weeks | Focus | Prerequisites |
|--------|-------|-------|---------------|
| Intro | 1-2 | Physical AI concepts | None |
| Module 1 | 3-5 | ROS 2 fundamentals | Python basics |
| Module 2 | 6-7 | Gazebo & Unity simulation | Module 1 |
| Module 3 | 8-10 | Isaac Sim, perception, RL | Modules 1-2 |
| Module 4 | 11-13 | VLA systems, capstone | Modules 1-3 |

### When to Attempt Assessments

- Complete all lessons in the module
- Feel comfortable with hands-on exercises
- Can explain concepts without referring to notes
- Each assessment takes 4-6 hours

## Hardware Decision Tree

### Path 1: Simulation-Only (Minimum Investment)

**Requirements:**
- RTX 4070 Ti (12GB VRAM) or better
- 32GB RAM minimum (64GB recommended)
- 500GB SSD
- Ubuntu 22.04 native

**Cost:** ~$1,500 (if building new PC)
**Suitable for:** Modules 1-4, full capstone in simulation

### Path 2: Cloud-Based (Pay-as-you-go)

**Requirements:**
- Any computer with internet
- AWS account (EC2 g5.2xlarge instances)

**Cost:** ~$1.20/hour = $36-60 for Module 3 (Isaac Sim)
**Suitable for:** All modules, Isaac Sim in cloud

### Path 3: Full Hardware (Physical Robot Deployment)

**Requirements:**
- Path 1 or 2 (simulation)
- Jetson Orin Nano 8GB ($499)
- Intel RealSense D435i ($200)
- IMU module ($30)
- Microphone array ($60)
- Optional: Unitree Go2 robot ($2,700 - shared lab access)

**Cost:** ~$789 (edge kit) + optional robot
**Suitable for:** Full course + sim-to-real deployment

### Which Path is Right for You?

- **Student/Hobbyist:** Path 1 (simulation-only) or Path 2 (cloud)
- **Professional/Researcher:** Path 3 (full hardware)
- **Institution/Lab:** Path 3 with shared robot access

## Getting Help

### Common Issues

**1. ROS 2 not found after installation**
- Solution: Source the setup file: `source /opt/ros/humble/setup.bash`

**2. Permission denied when running scripts**
- Solution: Make script executable: `chmod +x script_name.py`

**3. GPU drivers not working (NVIDIA)**
- Solution: Install proprietary drivers: `ubuntu-drivers install`

**4. Gazebo/Isaac Sim crashes**
- Solution: Update GPU drivers, check VRAM usage, reduce scene complexity

### Community Resources

- **ROS Discourse:** [discourse.ros.org](https://discourse.ros.org) - Official ROS community forum
- **GitHub Issues:** Report book issues or typos
- **Stack Overflow:** Tag questions with `ros2`, `gazebo`, `isaac-sim`
- **YouTube:** Official ROS 2 tutorials and community demos

### Contact for Feedback

- **Bug Reports:** Open issue on GitHub repository
- **Content Suggestions:** Discussion board or contact form
- **Technical Support:** ROS Discourse (community-driven)

## Next Steps

1. ✅ Complete environment setup (ROS 2 Humble)
2. ✅ Run "Hello ROS 2" exercise successfully
3. 📖 Read **Introduction to Physical AI** (Weeks 1-2 content)
4. 🚀 Begin **Module 1: The Robotic Nervous System**
5. 📊 Join community discussions and share your progress

**Ready to begin?** Head to [Introduction: What is Physical AI?](../docs/intro/what-is-physical-ai.md)

---

**Last Updated:** 2025-12-05
**Estimated Setup Time:** 1-2 hours
**Support:** [Community Forum](#) | [GitHub Issues](#)
