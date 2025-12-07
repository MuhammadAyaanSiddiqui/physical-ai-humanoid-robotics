---
sidebar_position: 1
---

# ROS 2 Humble Installation

## Overview

ROS 2 (Robot Operating System 2) is the industry-standard middleware for building robot applications. This guide covers installing ROS 2 Humble Hawksbill on Ubuntu 22.04 LTS, the recommended long-term support (LTS) version with support until May 2027.

## Why ROS 2 Humble?

- **LTS Support**: Guaranteed updates until 2027
- **NVIDIA Isaac Compatibility**: Official support for Isaac ROS
- **Community Adoption**: Largest ROS 2 user base
- **Ubuntu 22.04 LTS**: Aligns with widely-used LTS Ubuntu

## Prerequisites

### System Requirements

- **Operating System**: Ubuntu 22.04 Jammy Jellyfish (LTS)
- **RAM**: Minimum 4GB (8GB+ recommended)
- **Disk Space**: 10GB free space
- **Network**: Internet connection for package downloads

### Check Ubuntu Version

```bash
lsb_release -a
```

Expected output:
```
Distributor ID: Ubuntu
Description:    Ubuntu 22.04.x LTS
Release:        22.04
Codename:       jammy
```

## Installation Methods

### Method 1: Debian Packages (Recommended)

Easiest method for beginners. Installs pre-built binaries.

#### Step 1: Set Locale

Ensure UTF-8 locale support:

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

#### Step 2: Add ROS 2 APT Repository

```bash
# Add Universe repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### Step 3: Install ROS 2 Packages

```bash
# Update package index
sudo apt update
sudo apt upgrade

# Install ROS 2 Desktop (includes RViz, demos, tutorials)
sudo apt install ros-humble-desktop

# Or install ROS 2 Base (minimal, no GUI tools)
# sudo apt install ros-humble-ros-base
```

**Installation time**: 5-15 minutes depending on network speed

#### Step 4: Install Development Tools

```bash
# Install colcon (build tool)
sudo apt install python3-colcon-common-extensions

# Install rosdep (dependency management)
sudo apt install python3-rosdep2
sudo rosdep init
rosdep update
```

#### Step 5: Environment Setup

Add ROS 2 to your shell environment:

```bash
# Add to ~/.bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
printenv | grep -i ROS
```

Expected output should include:
```
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```

### Method 2: Building from Source (Advanced)

For developers who need bleeding-edge features or custom modifications.

:::warning
This method takes 1-2 hours and requires 20GB+ disk space. Only recommended for advanced users.
:::

[See official source installation guide](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)

## Verify Installation

### Test 1: Check ROS 2 Version

```bash
ros2 --version
```

Expected output:
```
ros2 cli version: 0.25.x
```

### Test 2: Run Talker Demo

Open a terminal and run:

```bash
ros2 run demo_nodes_cpp talker
```

Expected output:
```
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'
...
```

### Test 3: Run Listener Demo

Open a **second terminal** and run:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

Expected output:
```
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
...
```

**Success!** If both nodes communicate, ROS 2 is correctly installed.

## Common Issues

### Issue 1: "ros2: command not found"

**Solution**: Source the setup file:
```bash
source /opt/ros/humble/setup.bash
```

Add to `~/.bashrc` to make it permanent.

### Issue 2: GPG Key Error

**Solution**: Re-download the GPG key:
```bash
sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Issue 3: Package Not Found

**Solution**: Update package index:
```bash
sudo apt update
sudo apt upgrade
```

### Issue 4: rosdep Init Fails

**Solution**: Remove existing rosdep:
```bash
sudo rm -rf /etc/ros/rosdep
sudo rosdep init
rosdep update
```

## Recommended Tools

Install additional tools for development:

```bash
# Python dependencies
sudo apt install python3-pip python3-pytest python3-argcomplete

# VS Code ROS extension
code --install-extension ms-iot.vscode-ros

# RQt tools for visualization
sudo apt install ros-humble-rqt*

# TurtleSim (tutorial robot)
sudo apt install ros-humble-turtlesim
```

## Next Steps

Now that ROS 2 is installed:

1. Learn about [Nodes and Topics](./nodes-topics.md)
2. Explore [Services and Actions](./services-actions.md)
3. Master [CLI Tools](./cli-tools.md)
4. Build your [First Python Node](../ch2-python-rclpy/first-node.md)

## Resources

- [Official ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS Discourse](https://discourse.ros.org/) - Community forum
- [ROS Answers](https://answers.ros.org/) - Q&A site

---

**Prerequisites**: Ubuntu 22.04 LTS
**Estimated Time**: 30-45 minutes
**Learning Outcomes**:
- Install ROS 2 Humble on Ubuntu 22.04
- Configure development environment
- Verify installation with demo nodes
- Troubleshoot common issues
