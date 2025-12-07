# Isaac Sim Installation & Setup

## Overview

NVIDIA Isaac Sim is a robotics simulation platform built on NVIDIA Omniverse, providing photorealistic rendering, accurate physics simulation, and powerful AI capabilities for developing and testing robots in virtual environments before deploying to physical hardware.

**What You'll Learn**:
- Install Isaac Sim via Omniverse Launcher (local and cloud)
- Configure system requirements and GPU drivers
- Verify installation with a test scene
- Understand Isaac Sim workspace basics

**Prerequisites**:
- Completed Module 2 (Gazebo/Unity simulation)
- Ubuntu 22.04 or Windows 10/11
- NVIDIA RTX GPU (RTX 3070 or higher recommended) OR AWS cloud account
- 50GB+ free disk space

**Estimated Time**: 1-2 hours (depending on download speed)

---

## Learning Objectives

By the end of this lesson, you will be able to:

1. Install NVIDIA Omniverse Launcher and Isaac Sim
2. Configure CUDA drivers and verify GPU compatibility
3. Launch Isaac Sim and navigate the interface
4. Load and run sample robot scenes
5. Choose between local GPU and cloud deployment strategies

---

## System Requirements

### Local Installation (Recommended for Performance)

**Minimum Requirements**:
- **GPU**: NVIDIA RTX 2070 or higher (8GB VRAM)
- **CPU**: Intel Core i7 or AMD Ryzen 7 (8 cores)
- **RAM**: 32GB
- **Storage**: 50GB SSD (NVMe recommended)
- **OS**: Ubuntu 22.04 LTS or Windows 10/11
- **Driver**: NVIDIA Driver 525.xx or newer

**Recommended Configuration** (Digital Twin Workstation):
- **GPU**: NVIDIA RTX 4070 Ti (12GB VRAM) or RTX 4090 (24GB VRAM)
- **CPU**: Intel Core i9-13900K or AMD Ryzen 9 7950X
- **RAM**: 64GB DDR5
- **Storage**: 1TB NVMe SSD
- **OS**: Ubuntu 22.04 LTS (dual-boot or native)
- **Driver**: NVIDIA Driver 535.xx+ (latest stable)

:::tip Why RTX 4070 Ti+?
Isaac Sim leverages ray tracing (RTX cores) for photorealistic rendering and tensor cores for AI perception models. The RTX 4070 Ti offers excellent performance-per-dollar for robotics simulation with 12GB VRAM sufficient for most scenes.
:::

### Cloud Deployment (Alternative for Accessibility)

**AWS EC2 g5.2xlarge Instance**:
- **GPU**: NVIDIA A10G (24GB VRAM)
- **vCPUs**: 8
- **RAM**: 32GB
- **Storage**: 200GB GP3 SSD
- **Cost**: ~$1.20/hour (on-demand), ~$0.43/hour (spot instances)
- **Region**: us-east-1 or us-west-2 (lowest latency)

**Advantages**:
- No upfront hardware cost
- Access from any device
- Scalable compute resources
- Pay only for usage time

**Tradeoffs**:
- Monthly costs can exceed local GPU if used >100 hours
- Requires stable internet connection
- Latency for interactive simulation

:::note Cloud vs Local Decision
If you plan to use Isaac Sim >100 hours over 6 months, local GPU is more cost-effective. For short-term learning or experimentation, cloud deployment is ideal.
:::

---

## Part 1: GPU Driver Installation (Ubuntu 22.04)

### Step 1: Check Current NVIDIA Driver

```bash
nvidia-smi
```

**Expected Output**:
```
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 535.129.03   Driver Version: 535.129.03   CUDA Version: 12.2   |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  N/A |
|  0%   45C    P8    15W / 285W |    512MiB / 12288MiB |      2%      Default |
+-------------------------------+----------------------+----------------------+
```

If you see this output, your driver is already installed. **Skip to Part 2.**

If you get `Command 'nvidia-smi' not found`, proceed with driver installation:

### Step 2: Install NVIDIA Driver (if needed)

```bash
# Update package list
sudo apt update

# Install NVIDIA driver (Ubuntu automatically selects compatible version)
sudo ubuntu-drivers autoinstall

# Reboot to load driver
sudo reboot
```

After reboot, verify installation:

```bash
nvidia-smi
```

### Step 3: Install CUDA Toolkit (Required for Isaac Sim)

```bash
# Install CUDA 12.1 (compatible with Isaac Sim 2023.1.1+)
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get -y install cuda-12-1

# Add CUDA to PATH
echo 'export PATH=/usr/local/cuda-12.1/bin${PATH:+:${PATH}}' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-12.1/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.bashrc
source ~/.bashrc

# Verify CUDA installation
nvcc --version
```

**Expected Output**:
```
nvcc: NVIDIA (R) Cuda compiler driver
Copyright (c) 2005-2023 NVIDIA Corporation
Built on Mon_Apr__3_17:16:06_PDT_2023
Cuda compilation tools, release 12.1, V12.1.105
Build cuda_12.1.r12.1/compiler.32688072_0
```

---

## Part 2: Omniverse Launcher Installation

### Step 1: Download Omniverse Launcher

**For Linux (Ubuntu 22.04)**:

```bash
# Download Omniverse Launcher AppImage
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make it executable
chmod +x omniverse-launcher-linux.AppImage

# Run the launcher
./omniverse-launcher-linux.AppImage
```

**For Windows**:

1. Visit: https://www.nvidia.com/en-us/omniverse/download/
2. Click **Download Omniverse Launcher**
3. Run the installer (`omniverse-launcher-win.exe`)
4. Follow the installation wizard

### Step 2: Create NVIDIA Account

When you first launch Omniverse, you'll be prompted to sign in or create an NVIDIA account (free). This is required to access Isaac Sim.

1. Click **Sign In**
2. Create account at https://www.nvidia.com/account/ (if you don't have one)
3. Verify your email
4. Sign in to Omniverse Launcher

### Step 3: Install Isaac Sim from Exchange

Once signed in:

1. Click **Exchange** tab in Omniverse Launcher
2. Search for "**Isaac Sim**"
3. Click on **Isaac Sim** (not Isaac Sim Headless)
4. Click **Install**
5. Select installation directory (default: `~/Documents/Kit/Isaac-Sim` on Linux)
6. Wait for download and installation (~20GB download, 45GB installed)

:::caution Download Size
Isaac Sim is a large download (~20GB). Ensure you have a stable internet connection and 50GB+ free disk space before installing.
:::

**Installation Time Estimates**:
- Fast internet (100 Mbps+): 15-30 minutes
- Moderate internet (25-50 Mbps): 1-2 hours
- Slow internet (&lt;10 Mbps): 3-5 hours

---

## Part 3: First Launch & Workspace Setup

### Step 1: Launch Isaac Sim

1. In Omniverse Launcher, go to **Library** tab
2. Find **Isaac Sim** in your installed apps
3. Click **Launch**

**First Launch Notes**:
- Isaac Sim will compile shaders on first launch (5-10 minutes)
- A window may appear installing Python packages (automatic)
- You may see NVIDIA End User License Agreement (accept to continue)

### Step 2: Verify Installation with Sample Scene

Once Isaac Sim opens, you'll see the main interface:

1. **File** > **Open** > Navigate to sample scenes
2. Select: `Isaac-Sim/standalone_examples/api/omni.isaac.franka/franka_bin_example.usd`
3. Click **Open**

You should see a Franka Emika Panda robot arm with a bin of objects.

### Step 3: Run Your First Simulation

1. Click the **Play** button (▶) in the toolbar (or press `Space`)
2. Observe the robot arm picking and placing objects
3. Click **Stop** (■) to pause simulation

**Expected Behavior**:
- Robot arm moves smoothly to grasp objects
- Physics simulation runs at 60 FPS (check bottom-right corner)
- Objects fall naturally with gravity

:::tip Performance Check
If FPS is below 30, check:
- GPU driver is correctly installed (`nvidia-smi` in terminal)
- No other GPU-intensive apps are running
- RTX settings enabled: **Edit** > **Preferences** > **Rendering** > Enable Ray Tracing
:::

### Step 4: Explore Isaac Sim Interface

Key interface elements:

- **Viewport**: Main 3D rendering window (center)
- **Stage**: Scene hierarchy tree (left panel)
- **Property**: Object properties and physics parameters (right panel)
- **Content Browser**: Asset library and file browser (bottom)
- **Toolbar**: Simulation controls (top)

**Useful Camera Controls**:
- **Tumble**: Left-click + drag (rotate around focus)
- **Pan**: Middle-click + drag (move sideways)
- **Dolly**: Right-click + drag (zoom in/out)
- **Frame Selection**: Press `F` with object selected

---

## Part 4: Cloud Deployment (AWS EC2 Alternative)

### Prerequisites for Cloud Setup

- AWS account with billing enabled
- Familiarity with SSH and Linux command line

### Step 1: Launch EC2 Instance

1. Log in to AWS Console: https://console.aws.amazon.com/ec2/
2. Click **Launch Instance**
3. Configure:
   - **Name**: `isaac-sim-workstation`
   - **AMI**: Deep Learning AMI (Ubuntu 22.04) - has NVIDIA drivers pre-installed
   - **Instance Type**: `g5.2xlarge` (NVIDIA A10G GPU)
   - **Key Pair**: Create new or select existing SSH key
   - **Storage**: 200GB gp3 SSD (increase from default 100GB)
   - **Security Group**: Allow SSH (port 22) and VNC (port 5901) from your IP

4. Click **Launch Instance**
5. Wait ~5 minutes for instance to start

### Step 2: Connect to Instance

```bash
# Replace <your-key.pem> and <instance-ip> with your values
chmod 400 your-key.pem
ssh -i your-key.pem ubuntu@<instance-public-ip>
```

### Step 3: Install Omniverse Launcher on Cloud Instance

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install desktop environment (for VNC access)
sudo apt install ubuntu-desktop -y

# Install VNC server
sudo apt install tightvncserver -y

# Start VNC server
vncserver :1 -geometry 1920x1080 -depth 24

# Download and install Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
chmod +x omniverse-launcher-linux.AppImage

# Install Isaac Sim (headless mode for cloud)
# Follow Part 2 steps via VNC viewer
```

### Step 4: Access Isaac Sim via VNC

1. On local machine, create SSH tunnel:

```bash
ssh -i your-key.pem -L 5901:localhost:5901 ubuntu@<instance-public-ip>
```

2. Open VNC Viewer (download from https://www.realvnc.com/en/connect/download/viewer/)
3. Connect to: `localhost:5901`
4. Enter VNC password (set during `vncserver` setup)
5. Launch Isaac Sim from VNC desktop

:::caution Cloud Cost Management
Remember to **Stop** (not terminate) your EC2 instance when not in use. A g5.2xlarge instance costs ~$1.20/hour when running. Use AWS Budgets to set spending alerts.
:::

---

## Part 5: Troubleshooting Common Issues

### Issue 1: "NVIDIA driver not found" Error

**Symptom**: Isaac Sim fails to launch with driver error

**Solution**:
```bash
# Check driver installation
nvidia-smi

# If no output, reinstall driver
sudo ubuntu-drivers autoinstall
sudo reboot
```

### Issue 2: Low FPS (&lt;20 FPS)

**Symptom**: Simulation runs slowly, laggy viewport

**Solutions**:
1. Lower rendering quality:
   - **Edit** > **Preferences** > **Rendering** > Set **Anti-Aliasing** to **FXAA**
   - Disable **Ray Tracing** if not needed

2. Check GPU utilization:
```bash
watch -n 1 nvidia-smi
```
If GPU utilization is low (&lt;50%), there may be a CPU bottleneck.

3. Close other GPU applications (browsers with hardware acceleration, other simulations)

### Issue 3: Omniverse Launcher Won't Start

**Symptom**: AppImage crashes or nothing happens

**Solution (Linux)**:
```bash
# Install required dependencies
sudo apt install libfuse2 libnss3 libgconf-2-4 -y

# Run AppImage with verbose output
./omniverse-launcher-linux.AppImage --verbose
```

### Issue 4: Isaac Sim Installation Stuck

**Symptom**: Download progress bar frozen

**Solutions**:
1. Check internet connection stability
2. Restart Omniverse Launcher
3. Clear cache: **Settings** > **Cache** > **Clear All**
4. Try installing during off-peak hours (download servers may be congested)

### Issue 5: "Out of Memory" Error

**Symptom**: Isaac Sim crashes with CUDA out-of-memory error

**Solutions**:
1. Close Isaac Sim and relaunch
2. Load smaller scenes (avoid complex USD files initially)
3. Reduce texture quality: **Edit** > **Preferences** > **Rendering** > Lower **Texture Streaming Budget**
4. Upgrade to GPU with more VRAM (8GB minimum, 12GB+ recommended)

---

## Part 6: Next Steps

Now that Isaac Sim is installed, you're ready to:

1. **Explore Sample Scenes**: Navigate to `Isaac-Sim/standalone_examples/` to try different robot simulations
2. **Learn Isaac Sim Python API**: Run standalone Python scripts that control Isaac Sim programmatically
3. **Import ROS 2 Robots**: Bring your URDF models from Module 1 into Isaac Sim

**In the next lesson** (Photorealistic Environments), you'll learn:
- How to create custom environments and scenes
- Asset importing (CAD models, textures, props)
- Lighting and material setup for realism
- Scene composition and layout design

---

## Summary

In this lesson, you learned:

✅ How to check system requirements and install NVIDIA drivers
✅ How to install Omniverse Launcher and Isaac Sim
✅ How to launch Isaac Sim and run sample scenes
✅ How to deploy Isaac Sim on AWS cloud (alternative to local GPU)
✅ Common troubleshooting steps for installation issues

**Key Takeaways**:
- Isaac Sim requires RTX GPU (local) or cloud instance (g5.2xlarge)
- Installation is ~20GB download, 45GB installed
- First launch compiles shaders (5-10 minutes)
- Cloud deployment costs ~$1.20/hour but eliminates upfront hardware cost

---

## Hands-On Exercise

**Objective**: Verify your Isaac Sim installation by running a humanoid robot simulation

### Exercise Steps

1. Launch Isaac Sim
2. **File** > **Open**
3. Navigate to: `Isaac-Sim/standalone_examples/api/omni.isaac.core/carter_warehouse.usd`
4. Click **Play** (▶)
5. Observe the Carter robot navigating the warehouse
6. Take a screenshot of the simulation running
7. Check FPS in bottom-right corner (should be 30+ FPS)

**Acceptance Criteria**:
- Isaac Sim launches without errors
- Sample scene loads successfully
- Simulation runs at stable FPS (>30)
- Physics behaves realistically (objects don't fly off unexpectedly)

**Submit**: Screenshot showing Isaac Sim interface with Carter robot simulation running and FPS counter visible

---

## Additional Resources

- **Official Documentation**: https://docs.omniverse.nvidia.com/isaacsim/latest/index.html
- **Isaac Sim Forum**: https://forums.developer.nvidia.com/c/omniverse/isaac-sim/69
- **Video Tutorials**: https://www.youtube.com/c/NVIDIAOmniverse
- **Sample Code Repository**: https://github.com/NVIDIA-Omniverse/IsaacSim-ros_workspaces
- **ROS 2 Integration Guide**: https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/index.html

:::info Need Help?
If you encounter issues not covered in troubleshooting, visit the Isaac Sim forum or check Appendix B: Troubleshooting for more solutions.
:::
