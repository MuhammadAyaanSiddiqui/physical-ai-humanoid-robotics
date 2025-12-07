# ROS 2 Workspace Setup with Colcon

**Module**: The Robotic Nervous System
**Chapter**: Package Development
**Estimated Time**: 2-3 hours
**Difficulty**: Beginner to Intermediate

## Prerequisites

- ROS 2 Humble installed and sourced
- Basic understanding of ROS 2 nodes and topics
- Familiarity with terminal/command line
- Python or C++ programming knowledge

## Learning Objectives

By the end of this lesson, you will be able to:

- Understand ROS 2 workspace structure and overlay concept
- Create and organize a colcon workspace
- Build packages using colcon
- Source overlay workspaces correctly
- Manage dependencies with package.xml
- Use colcon commands effectively
- Debug common workspace issues

## What is a ROS 2 Workspace?

A **ROS 2 workspace** is a directory structure where you develop, build, and organize ROS 2 packages.

### Key Concepts

**1. Overlay vs Underlay**:
- **Underlay**: The base ROS 2 installation (`/opt/ros/humble`)
- **Overlay**: Your workspace built on top of the underlay

**2. Source Space**:
- Contains package source code
- Typically in `src/` directory

**3. Build Space**:
- Contains build artifacts
- Created by colcon in `build/` directory

**4. Install Space**:
- Contains installed executables and libraries
- Created by colcon in `install/` directory

**5. Log Space**:
- Contains build logs
- Created in `log/` directory

## Workspace Structure

```
my_robot_ws/
├── src/                    # Source space (your packages)
│   ├── package_1/
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── package_1/
│   │       └── node.py
│   └── package_2/
│       ├── package.xml
│       ├── CMakeLists.txt
│       └── src/
│           └── node.cpp
├── build/                  # Build space (auto-generated)
│   ├── package_1/
│   └── package_2/
├── install/                # Install space (auto-generated)
│   ├── setup.bash
│   ├── package_1/
│   └── package_2/
└── log/                    # Log space (auto-generated)
    └── build_*/
```

## Creating a Workspace

### Step 1: Create Workspace Directory

```bash
# Create workspace root
mkdir -p ~/robot_ws/src

# Navigate to workspace
cd ~/robot_ws
```

**Explanation**:
- `~/robot_ws` is the workspace root
- `src/` will contain your packages
- `build/`, `install/`, `log/` will be auto-created by colcon

### Step 2: Source ROS 2 (Underlay)

```bash
source /opt/ros/humble/setup.bash
```

**Important**: Always source the underlay before building!

### Step 3: Build Empty Workspace

```bash
cd ~/robot_ws
colcon build
```

**Output**:
```
Starting >>> [no packages]
Finished <<< [0 packages built, 0 packages skipped]

Summary: 0 packages finished [0.15s]
```

Even with no packages, this creates the workspace structure.

### Step 4: Source the Overlay

```bash
source ~/robot_ws/install/setup.bash
```

**Now your workspace is active!**

## Understanding Colcon

**Colcon** (collective construction) is the ROS 2 build tool that replaced `catkin` from ROS 1.

### Why Colcon?

- **Language-agnostic**: Builds Python, C++, and mixed packages
- **Parallel builds**: Builds multiple packages simultaneously
- **Isolated builds**: Each package builds independently
- **Flexible**: Supports various build systems (ament_cmake, ament_python, cmake, setuptools)

### Basic Colcon Commands

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select my_package

# Build with output to console
colcon build --symlink-install --event-handlers console_direct+

# Clean build (rebuild from scratch)
rm -rf build/ install/ log/
colcon build

# Build up to a specific package
colcon build --packages-up-to my_package

# Build and run tests
colcon test

# Show test results
colcon test-result --all
```

## Creating Your First Package

### Python Package

**Step 1**: Navigate to src directory:
```bash
cd ~/robot_ws/src
```

**Step 2**: Create package:
```bash
ros2 pkg create --build-type ament_python my_python_pkg \
  --dependencies rclpy std_msgs
```

**Generated Structure**:
```
my_python_pkg/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── my_python_pkg
└── my_python_pkg/
    └── __init__.py
```

**Step 3**: Add a node:
```bash
cd my_python_pkg/my_python_pkg
cat > my_node.py << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.publisher = self.create_publisher(String, 'my_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('My node started!')

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from my_python_pkg!'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

chmod +x my_node.py
```

**Step 4**: Edit `setup.py` to add entry point:
```python
from setuptools import setup

package_name = 'my_python_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='My first ROS 2 package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_python_pkg.my_node:main',
        ],
    },
)
```

**Step 5**: Build and source:
```bash
cd ~/robot_ws
colcon build --packages-select my_python_pkg
source install/setup.bash
```

**Step 6**: Run the node:
```bash
ros2 run my_python_pkg my_node
```

### C++ Package

**Step 1**: Create package:
```bash
cd ~/robot_ws/src
ros2 pkg create --build-type ament_cmake my_cpp_pkg \
  --dependencies rclcpp std_msgs
```

**Generated Structure**:
```
my_cpp_pkg/
├── package.xml
├── CMakeLists.txt
├── include/
│   └── my_cpp_pkg/
└── src/
```

**Step 2**: Add a node:
```bash
cd my_cpp_pkg/src
cat > my_node.cpp << 'EOF'
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10);
        timer_ = this->create_wall_timer(
            1s, std::bind(&MyNode::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "My node started!");
    }

private:
    void timer_callback() {
        auto message = std_msgs::msg::String();
        message.data = "Hello from my_cpp_pkg!";
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNode>());
    rclcpp::shutdown();
    return 0;
}
EOF
```

**Step 3**: Edit `CMakeLists.txt` to add executable:
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp std_msgs)

# Install executables
install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

**Step 4**: Build and run:
```bash
cd ~/robot_ws
colcon build --packages-select my_cpp_pkg
source install/setup.bash
ros2 run my_cpp_pkg my_node
```

## Package Dependencies

### package.xml

Every ROS 2 package has a `package.xml` file defining metadata and dependencies.

**Example**:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_pkg</name>
  <version>0.0.1</version>
  <description>My robot control package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <!-- Build tool dependency -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Runtime dependencies -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>

  <!-- Build-only dependencies (C++ only) -->
  <build_depend>rosidl_default_generators</build_depend>

  <!-- Execution-only dependencies -->
  <exec_depend>ros2launch</exec_depend>

  <!-- Test dependencies -->
  <test_depend>ament_pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Dependency Types

| Tag | Purpose |
|-----|---------|
| `<buildtool_depend>` | Build system (ament_cmake, ament_python) |
| `<depend>` | Both build and runtime dependency |
| `<build_depend>` | Only needed during build |
| `<exec_depend>` | Only needed at runtime |
| `<test_depend>` | Only needed for tests |

### Installing Dependencies

**Check for missing dependencies**:
```bash
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y
```

**What this does**:
- `--from-paths src`: Check all packages in src/
- `--ignore-src`: Don't try to install packages you're developing
- `-r`: Continue even if some dependencies fail
- `-y`: Auto-approve installation

## Sourcing Workspaces

### The Overlay System

ROS 2 uses **workspace overlays** stacked on top of each other:

```
┌─────────────────────────────┐
│   Your Workspace Overlay    │  ~/robot_ws/install/setup.bash
├─────────────────────────────┤
│   Base ROS 2 Underlay       │  /opt/ros/humble/setup.bash
└─────────────────────────────┘
```

### Sourcing Order Matters!

**Correct Order**:
```bash
# 1. Source underlay first
source /opt/ros/humble/setup.bash

# 2. Build your workspace
cd ~/robot_ws
colcon build

# 3. Source your overlay
source ~/robot_ws/install/setup.bash
```

**Wrong Order** (will fail):
```bash
# DON'T DO THIS!
source ~/robot_ws/install/setup.bash  # Overlay first - ERROR!
source /opt/ros/humble/setup.bash     # Underlay second - too late
```

### Multiple Workspaces

You can stack multiple overlays:

```bash
# Base ROS 2
source /opt/ros/humble/setup.bash

# Common packages workspace
source ~/common_ws/install/setup.bash

# Your specific project
source ~/my_project_ws/install/setup.bash
```

**Package priority**: Later sourced workspaces override earlier ones.

### Automatic Sourcing (Recommended)

Add to `~/.bashrc`:

```bash
# ROS 2 setup
source /opt/ros/humble/setup.bash

# Your workspace (if it exists)
if [ -f ~/robot_ws/install/setup.bash ]; then
  source ~/robot_ws/install/setup.bash
fi

# Helpful aliases
alias robot_ws='cd ~/robot_ws'
alias build_ws='cd ~/robot_ws && colcon build && source install/setup.bash'
```

**Reload**:
```bash
source ~/.bashrc
```

## Colcon Build Options

### Symlink Install (Recommended for Python)

```bash
colcon build --symlink-install
```

**Benefits**:
- Python file changes don't require rebuild
- Faster iteration during development
- No need to re-source after Python edits

**How it works**: Creates symbolic links instead of copying files.

### Parallel Builds

```bash
# Use 4 parallel jobs
colcon build --parallel-workers 4

# Use all CPU cores
colcon build --parallel-workers $(nproc)
```

### Event Handlers (Better Output)

```bash
# Show all output during build
colcon build --event-handlers console_direct+

# Colorized output with status
colcon build --event-handlers console_cohesion+
```

### Package Selection

```bash
# Build single package
colcon build --packages-select my_pkg

# Build multiple packages
colcon build --packages-select pkg1 pkg2 pkg3

# Build package and dependencies
colcon build --packages-up-to my_pkg

# Skip specific packages
colcon build --packages-skip unwanted_pkg
```

### CMake Arguments (C++ packages)

```bash
# Debug build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Release build (optimized)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# With tests
colcon build --cmake-args -DBUILD_TESTING=ON
```

## Common Workflows

### Daily Development Workflow

**1. Morning setup**:
```bash
cd ~/robot_ws
source /opt/ros/humble/setup.bash
```

**2. Make changes to Python code**:
```bash
# Edit files in src/my_pkg/my_pkg/
nano src/my_pkg/my_pkg/my_node.py

# No rebuild needed if using --symlink-install!
ros2 run my_pkg my_node
```

**3. Make changes to C++ code**:
```bash
# Edit files in src/my_cpp_pkg/src/
nano src/my_cpp_pkg/src/my_node.cpp

# Rebuild required
colcon build --packages-select my_cpp_pkg
source install/setup.bash
ros2 run my_cpp_pkg my_node
```

**4. Add new dependencies**:
```bash
# Edit package.xml to add dependency
nano src/my_pkg/package.xml

# Install new dependencies
rosdep install --from-paths src --ignore-src -r -y

# Rebuild
colcon build --packages-select my_pkg
```

### Testing Workflow

```bash
# Build with tests
colcon build --cmake-args -DBUILD_TESTING=ON

# Run tests for all packages
colcon test

# Run tests for specific package
colcon test --packages-select my_pkg

# Show test results
colcon test-result --all

# Verbose test output
colcon test-result --verbose
```

### Clean Build Workflow

When things go wrong:

```bash
cd ~/robot_ws

# Option 1: Clean specific package
rm -rf build/my_pkg install/my_pkg
colcon build --packages-select my_pkg

# Option 2: Clean everything and rebuild
rm -rf build/ install/ log/
colcon build

# Option 3: Nuclear option (including source changes)
rm -rf build/ install/ log/
git clean -fdx src/  # WARNING: Removes all untracked files!
colcon build
```

## Best Practices

### 1. Workspace Organization

**Good Structure**:
```
~/ros2_workspaces/
├── common_ws/          # Shared libraries
│   └── src/
│       ├── my_utils/
│       └── my_msgs/
└── robot_ws/           # Robot-specific code
    └── src/
        ├── robot_control/
        └── robot_vision/
```

### 2. Git and Version Control

**Add to `.gitignore`**:
```bash
cd ~/robot_ws
cat > .gitignore << 'EOF'
build/
install/
log/
*.pyc
__pycache__/
.vscode/
EOF

# Initialize git (only for src/)
git init
git add src/ .gitignore
git commit -m "Initial commit"
```

**Only version control `src/`!** Never commit `build/`, `install/`, or `log/`.

### 3. Package Naming Conventions

- Use lowercase with underscores: `my_robot_pkg`
- Be descriptive: `humanoid_control` not `ctrl`
- Prefix related packages: `myrobot_control`, `myrobot_vision`

### 4. Dependency Management

- Declare ALL dependencies in `package.xml`
- Use `rosdep` to install system dependencies
- Keep dependencies minimal (don't depend on what you don't use)

### 5. Build Speed Optimization

```bash
# Use symlink install for Python packages
colcon build --symlink-install

# Use ccache for C++ (install: sudo apt install ccache)
colcon build --cmake-args -DCMAKE_CXX_COMPILER_LAUNCHER=ccache

# Limit parallel jobs if low on RAM
colcon build --parallel-workers 2
```

## Troubleshooting

### Issue 1: "Package not found"

**Symptoms**: `ros2 run my_pkg my_node` fails

**Checklist**:
1. **Built the package?**
   ```bash
   colcon build --packages-select my_pkg
   ```

2. **Sourced the workspace?**
   ```bash
   source ~/robot_ws/install/setup.bash
   ```

3. **Entry point added?** (Python packages)
   - Check `setup.py` has the node in `entry_points`

4. **Installed executable?** (C++ packages)
   - Check `CMakeLists.txt` has `install(TARGETS ...)`

### Issue 2: Build Fails with Dependency Errors

**Symptoms**: `CMake Error: Could not find package X`

**Fix**:
```bash
# Install missing dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Rebuild
colcon build
```

### Issue 3: Changes Not Reflected

**Symptoms**: Code changes don't appear when running node

**Python**:
- Did you use `--symlink-install`? If not, rebuild:
  ```bash
  colcon build --symlink-install --packages-select my_pkg
  ```

**C++**:
- C++ always requires rebuild:
  ```bash
  colcon build --packages-select my_pkg
  source install/setup.bash
  ```

### Issue 4: Multiple Nodes with Same Name

**Symptoms**: "Node name already exists" warning

**Cause**: Sourced multiple overlays with conflicting packages

**Fix**:
```bash
# Start fresh terminal
# Source workspaces in correct order
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash  # Only once!
```

### Issue 5: setup.bash Not Found

**Symptoms**: `source install/setup.bash` fails

**Cause**: Haven't built the workspace yet

**Fix**:
```bash
cd ~/robot_ws
colcon build
# Now it exists:
source install/setup.bash
```

## Advanced Topics

### Mixed Python/C++ Packages

You can have both in one package:

**package.xml**:
```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<buildtool_depend>ament_cmake_python</buildtool_depend>
<depend>rclcpp</depend>
<depend>rclpy</depend>
```

**CMakeLists.txt** (add Python support):
```cmake
find_package(ament_cmake_python REQUIRED)

# C++ executables
add_executable(cpp_node src/cpp_node.cpp)

# Python modules
ament_python_install_package(${PROJECT_NAME})
```

### Custom Messages in Workspace

**Step 1**: Create message package:
```bash
ros2 pkg create --build-type ament_cmake my_msgs
cd my_msgs
mkdir msg
```

**Step 2**: Define message (`msg/CustomMsg.msg`):
```
string name
int32 value
float64 timestamp
```

**Step 3**: Edit `package.xml`:
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

**Step 4**: Edit `CMakeLists.txt`:
```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMsg.msg"
)
```

**Step 5**: Build:
```bash
cd ~/robot_ws
colcon build --packages-select my_msgs
source install/setup.bash
```

**Step 6**: Use in other packages:
```xml
<!-- In dependent package's package.xml -->
<depend>my_msgs</depend>
```

```python
# In Python code
from my_msgs.msg import CustomMsg
```

## Useful Colcon Commands Summary

```bash
# Build
colcon build                                    # Build all
colcon build --symlink-install                  # Symlink install
colcon build --packages-select pkg              # Build one package
colcon build --packages-up-to pkg               # Build with dependencies

# Test
colcon test                                     # Run tests
colcon test-result --all                        # Show results
colcon test-result --verbose                    # Detailed results

# Info
colcon list                                     # List packages
colcon list --packages-up-to pkg                # Show dependencies
colcon graph                                    # Show dependency graph

# Clean
colcon clean workspace                          # Remove build/install/log
colcon clean packages --packages-select pkg     # Clean one package
```

## Practice Exercises

### Exercise 1: Create Multi-Package Workspace

1. Create workspace with 3 packages:
   - `my_msgs`: Custom messages
   - `sensor_node`: Publisher using custom messages
   - `processor_node`: Subscriber processing messages

2. Set up dependencies correctly
3. Build and test the system

### Exercise 2: Troubleshoot Broken Workspace

Given a workspace with missing dependencies:
1. Identify missing dependencies using build errors
2. Add them to `package.xml`
3. Use `rosdep` to install
4. Successfully build

### Exercise 3: Optimize Build Performance

1. Measure baseline build time
2. Apply `--symlink-install`
3. Use `ccache` (C++ packages)
4. Adjust `--parallel-workers`
5. Compare results

## Key Takeaways

- **Workspace structure**: src/ (your code), build/ (artifacts), install/ (output), log/ (logs)
- **Overlay system**: Your workspace overlays the base ROS 2 installation
- **Colcon** is the build tool; use `colcon build` to compile packages
- **Source order matters**: Underlay first, then overlay
- **Symlink install** speeds up Python development (no rebuild needed)
- **package.xml** declares dependencies; use `rosdep` to install them
- **Git best practice**: Only version control `src/`, ignore `build/install/log/`

## What's Next?

Now that you can create and manage workspaces:

- **Next Lesson**: [Launch Files](./launch-files.md) - Automate multi-node startup
- **Related**: [Debugging](./debugging.md) - Troubleshoot ROS 2 issues

## Further Reading

- [Colcon Documentation](https://colcon.readthedocs.io/)
- [ROS 2 Package Creation Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [rosdep Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)

---

**Checkpoint**: You can now create, build, and manage ROS 2 workspaces!
