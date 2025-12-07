# ROS 2 Humble Installation on Ubuntu 22.04

**Module**: The Robotic Nervous System
**Chapter**: ROS 2 Fundamentals
**Estimated Time**: 1-2 hours
**Difficulty**: Beginner

## Prerequisites

Before installing ROS 2 Humble, ensure you have:

- Ubuntu 22.04 LTS (Jammy Jellyfish) installed - native or dual-boot recommended
- At least 20GB of free disk space
- Active internet connection
- Basic familiarity with Linux terminal commands

:::note
While WSL2 (Windows Subsystem for Linux) can run ROS 2, it has limitations with GUI applications and hardware access. Native Ubuntu or dual-boot is recommended for this course.
:::

## Learning Objectives

By the end of this lesson, you will be able to:

- Install ROS 2 Humble Hawksbill on Ubuntu 22.04
- Understand the ROS 2 directory structure and environment setup
- Verify your ROS 2 installation with test commands
- Source ROS 2 automatically on terminal startup

## What is ROS 2 Humble?

ROS 2 Humble Hawksbill is a Long-Term Support (LTS) release of the Robot Operating System 2, supported until May 2027. It provides:

- Modern middleware architecture using DDS (Data Distribution Service)
- Support for real-time systems
- Cross-platform compatibility (Linux, Windows, macOS)
- Enhanced security features
- Python 3.10+ and C++17 support

Humble is the recommended version for educational use due to its stability and extensive community support.

## Installation Methods

There are two primary ways to install ROS 2 Humble:

1. **Debian packages** (Recommended) - Quick and easy, suitable for beginners
2. **Build from source** - For advanced users needing custom configurations

This guide focuses on the Debian package installation method.

## Step 1: Set Up Locale

Ensure your system supports UTF-8 encoding:

```bash
# Check current locale settings
locale

# Install language support packages
sudo apt update && sudo apt install -y locales

# Generate UTF-8 locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Set environment variable
export LANG=en_US.UTF-8

# Verify locale is set correctly
locale
```

**Expected Output**: You should see `LANG=en_US.UTF-8` and `LC_ALL=en_US.UTF-8`.

## Step 2: Add ROS 2 Repository

Add the ROS 2 apt repository to your system:

```bash
# Install curl if not already installed
sudo apt install -y software-properties-common curl

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

## Step 3: Update Package Index

Update your system's package index to include the ROS 2 repository:

```bash
sudo apt update
sudo apt upgrade -y
```

:::caution
If you see warnings about GPG keys, repeat Step 2 to ensure the key was added correctly.
:::

## Step 4: Install ROS 2 Humble Desktop

Install the full ROS 2 Humble Desktop package, which includes:

- ROS 2 core libraries
- RViz (3D visualization tool)
- Demos and tutorials
- rqt (Qt-based GUI tools)

```bash
# Install ROS 2 Humble Desktop (Full installation)
sudo apt install -y ros-humble-desktop

# Install development tools (optional but recommended)
sudo apt install -y ros-dev-tools
```

This installation will take 5-10 minutes depending on your internet speed.

**Package Breakdown**:
- `ros-humble-desktop`: Complete ROS 2 installation with GUI tools (~2GB)
- `ros-dev-tools`: Additional build tools like colcon, rosdep, vcstool

:::tip Alternative: Minimal Installation
If disk space is limited, install the base package instead:
```bash
sudo apt install -y ros-humble-ros-base
```
This excludes GUI tools like RViz but includes all core ROS 2 functionality (~500MB).
:::

## Step 5: Source the ROS 2 Environment

To use ROS 2 commands, you must "source" the setup script in each terminal:

```bash
# Source ROS 2 Humble environment
source /opt/ros/humble/setup.bash
```

**What does sourcing do?**
- Adds ROS 2 executables to your `$PATH`
- Sets environment variables like `ROS_DISTRO` and `ROS_VERSION`
- Configures Python paths for ROS 2 packages

### Make Sourcing Automatic

To avoid sourcing manually every time, add the command to your `.bashrc` file:

```bash
# Add ROS 2 sourcing to .bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Reload .bashrc to apply changes
source ~/.bashrc
```

Now every new terminal will automatically have ROS 2 configured.

## Step 6: Verify Installation

Test your ROS 2 installation with these commands:

### Check ROS 2 Version

```bash
ros2 --version
```

**Expected Output**:
```
ros2 cli version: 0.18.5
```

### Check Environment Variables

```bash
printenv | grep -i ROS
```

**Expected Output** (example):
```
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```

### Run a Demo Node

Test ROS 2 functionality by running the talker-listener demo:

```bash
# Terminal 1: Run talker node
ros2 run demo_nodes_cpp talker
```

Open a new terminal and run:

```bash
# Terminal 2: Run listener node
ros2 run demo_nodes_cpp listener
```

**Expected Output**:

**Terminal 1 (talker)**:
```
[INFO] [talker]: Publishing: 'Hello World: 0'
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [talker]: Publishing: 'Hello World: 2'
...
```

**Terminal 2 (listener)**:
```
[INFO] [listener]: I heard: [Hello World: 0]
[INFO] [listener]: I heard: [Hello World: 1]
[INFO] [listener]: I heard: [Hello World: 2]
...
```

Press `Ctrl+C` in both terminals to stop the nodes.

:::success Congratulations!
If you see these messages, your ROS 2 Humble installation is working correctly!
:::

## Understanding the ROS 2 Directory Structure

ROS 2 Humble is installed in `/opt/ros/humble/`. Key directories:

```
/opt/ros/humble/
├── bin/              # Executable files (ros2 CLI, node executables)
├── include/          # C++ header files
├── lib/              # Compiled libraries and Python packages
├── share/            # Package resources (launch files, configs, URDFs)
└── setup.bash        # Environment setup script
```

## Common Installation Issues

### Issue 1: GPG Key Error

**Error**: `GPG error: ... The following signatures couldn't be verified`

**Solution**:
```bash
# Re-add the GPG key
sudo rm /usr/share/keyrings/ros-archive-keyring.gpg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo apt update
```

### Issue 2: Package Not Found

**Error**: `Unable to locate package ros-humble-desktop`

**Solution**:
```bash
# Verify Ubuntu version
lsb_release -a  # Should show Ubuntu 22.04

# Check repository was added correctly
cat /etc/apt/sources.list.d/ros2.list

# Update package index
sudo apt update
```

### Issue 3: ROS 2 Commands Not Found

**Error**: `ros2: command not found`

**Solution**:
```bash
# Source the setup script
source /opt/ros/humble/setup.bash

# Verify sourcing worked
echo $ROS_DISTRO  # Should output: humble
```

If still not working, check if installation completed successfully:
```bash
ls /opt/ros/humble/  # Should show bin/, lib/, share/, etc.
```

### Issue 4: Python Import Errors

**Error**: `ModuleNotFoundError: No module named 'rclpy'`

**Solution**:
```bash
# Reinstall ROS 2 Python packages
sudo apt install --reinstall ros-humble-rclpy

# Ensure Python 3.10 is default
python3 --version  # Should be 3.10.x

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
```

## Additional Tools (Optional)

Install these tools for enhanced ROS 2 development:

### Colcon Build Tool

```bash
# Install colcon (for building ROS 2 packages)
sudo apt install -y python3-colcon-common-extensions

# Add colcon auto-completion
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
source ~/.bashrc
```

### ROS 2 Development Dependencies

```bash
# Install rosdep (dependency management tool)
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update
```

### Python Development Tools

```bash
# Install Python tools for ROS 2 development
sudo apt install -y python3-pip python3-pytest python3-pytest-cov
pip3 install argcomplete
```

## Quick Reference Card

Keep these commands handy:

| Command | Purpose |
|---------|---------|
| `source /opt/ros/humble/setup.bash` | Activate ROS 2 environment |
| `ros2 --version` | Check ROS 2 version |
| `ros2 topic list` | List active topics |
| `ros2 node list` | List running nodes |
| `ros2 pkg list` | List installed packages |
| `ros2 run <package> <executable>` | Run a node |
| `ros2 doctor` | Diagnose ROS 2 setup issues |

## What's Next?

Now that you have ROS 2 Humble installed, you're ready to learn the core concepts:

- **Next Lesson**: [Nodes and Topics](./nodes-topics.md) - Learn how ROS 2 nodes communicate
- **Practice**: Try running other demo nodes: `ros2 run demo_nodes_py listener`, `ros2 run turtlesim turtlesim_node`

## Further Reading

- [Official ROS 2 Humble Documentation](https://docs.ros.org/en/humble/index.html)
- [ROS 2 Installation Troubleshooting](https://docs.ros.org/en/humble/Installation/Troubleshooting.html)
- [ROS 2 Concepts Overview](https://docs.ros.org/en/humble/Concepts.html)

---

**Checkpoint**: You've successfully installed ROS 2 Humble and verified it works! You're now ready to build your first robot programs.
