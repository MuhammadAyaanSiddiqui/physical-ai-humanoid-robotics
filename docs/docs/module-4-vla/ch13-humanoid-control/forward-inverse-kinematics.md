# Forward and Inverse Kinematics for Humanoid Robots

## Learning Objectives

By the end of this lesson, you will be able to:

- Understand forward and inverse kinematics (FK/IK) for robotic arms
- Implement FK using Denavit-Hartenberg (DH) parameters
- Use KDL (Kinematics and Dynamics Library) for IK solving
- Optimize IK solutions with TRAC-IK for humanoid robots
- Integrate kinematics solvers with ROS 2 MoveIt
- Handle singularities and joint limits

---

## Prerequisites

**Required Knowledge**:
- Linear algebra (transformation matrices, rotations)
- Python and C++ basics
- ROS 2 fundamentals

**Required Software**:
- ROS 2 Humble
- Python 3.10+
- KDL library (`sudo apt install ros-humble-orocos-kdl`)
- TRAC-IK (`sudo apt install ros-humble-trac-ik`)
- MoveIt 2

**Estimated Time**: 4-5 hours

---

## Introduction

**Forward Kinematics (FK)**: Given joint angles → Calculate end-effector pose
**Inverse Kinematics (IK)**: Given desired end-effector pose → Calculate joint angles

**Example**:
- **FK**: Arm joints at [30°, 45°, 60°] → Hand position (x=0.5m, y=0.3m, z=0.8m)
- **IK**: Want hand at (x=0.5m, y=0.3m, z=0.8m) → Need joints at [30°, 45°, 60°]

**Why IK is Hard**:
- **No unique solution**: Multiple joint configurations can reach same pose
- **Singularities**: Some poses are unreachable
- **Joint limits**: Physical constraints on robot movement
- **Computational complexity**: Solving nonlinear equations

---

## Part 1: Forward Kinematics

### Step 1: Denavit-Hartenberg (DH) Parameters

DH parameters define robot kinematics with 4 values per joint:

| Parameter | Symbol | Description |
|-----------|--------|-------------|
| Link length | a | Distance along X-axis |
| Link twist | α (alpha) | Rotation around X-axis |
| Link offset | d | Distance along Z-axis |
| Joint angle | θ (theta) | Rotation around Z-axis |

**Example: 3-DOF planar arm**:

| Joint | a (m) | α (rad) | d (m) | θ (rad) |
|-------|-------|---------|-------|---------|
| 1 | 0.5 | 0 | 0 | θ₁ |
| 2 | 0.3 | 0 | 0 | θ₂ |
| 3 | 0.2 | 0 | 0 | θ₃ |

---

### Step 2: FK Implementation in Python

```python
#!/usr/bin/env python3
import numpy as np

def dh_transform(a, alpha, d, theta):
    """
    Compute DH transformation matrix

    Args:
        a: Link length (m)
        alpha: Link twist (rad)
        d: Link offset (m)
        theta: Joint angle (rad)

    Returns:
        4x4 transformation matrix
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    T = np.array([
        [ct,    -st*ca,  st*sa,   a*ct],
        [st,     ct*ca, -ct*sa,   a*st],
        [0,      sa,     ca,      d   ],
        [0,      0,      0,       1   ]
    ])

    return T

def forward_kinematics(dh_params, joint_angles):
    """
    Compute FK for robot arm

    Args:
        dh_params: List of (a, alpha, d) for each joint
        joint_angles: List of theta values (rad)

    Returns:
        4x4 end-effector pose matrix
    """
    T = np.eye(4)  # Start with identity

    for (a, alpha, d), theta in zip(dh_params, joint_angles):
        T_i = dh_transform(a, alpha, d, theta)
        T = T @ T_i  # Chain transformations

    return T

# Example: 3-DOF planar arm
dh_params = [
    (0.5, 0, 0),  # Joint 1
    (0.3, 0, 0),  # Joint 2
    (0.2, 0, 0)   # Joint 3
]

joint_angles = [np.pi/6, np.pi/4, np.pi/3]  # 30°, 45°, 60°

T_end = forward_kinematics(dh_params, joint_angles)

# Extract position
x, y, z = T_end[0:3, 3]
print(f"End-effector position: x={x:.3f}, y={y:.3f}, z={z:.3f}")

# Extract orientation (roll-pitch-yaw)
def rotation_to_euler(R):
    """Convert rotation matrix to Euler angles (XYZ convention)"""
    roll = np.arctan2(R[2, 1], R[2, 2])
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw

roll, pitch, yaw = rotation_to_euler(T_end[0:3, 0:3])
print(f"Orientation: roll={np.degrees(roll):.1f}°, pitch={np.degrees(pitch):.1f}°, yaw={np.degrees(yaw):.1f}°")
```

---

## Part 2: Inverse Kinematics with KDL

### Step 1: Install and Setup KDL

```bash
# Install KDL
sudo apt install ros-humble-orocos-kdl ros-humble-python-orocos-kdl

# Install Python bindings
pip3 install pykdl-utils
```

---

### Step 2: Define Robot Chain in KDL

```python
#!/usr/bin/env python3
from PyKDL import *
import numpy as np

def create_simple_arm():
    """Create 3-DOF robot arm chain"""
    chain = Chain()

    # Joint 1: Revolute around Z, length 0.5m along X
    joint1 = Joint(Joint.RotZ)
    frame1 = Frame(Rotation.Identity(), Vector(0.5, 0, 0))
    segment1 = Segment(joint1, frame1)
    chain.addSegment(segment1)

    # Joint 2: Revolute around Z, length 0.3m along X
    joint2 = Joint(Joint.RotZ)
    frame2 = Frame(Rotation.Identity(), Vector(0.3, 0, 0))
    segment2 = Segment(joint2, frame2)
    chain.addSegment(segment2)

    # Joint 3: Revolute around Z, length 0.2m along X
    joint3 = Joint(Joint.RotZ)
    frame3 = Frame(Rotation.Identity(), Vector(0.2, 0, 0))
    segment3 = Segment(joint3, frame3)
    chain.addSegment(segment3)

    return chain

# Create robot
robot_chain = create_simple_arm()
print(f"Robot has {robot_chain.getNrOfJoints()} joints")
```

---

### Step 3: Solve IK with KDL

```python
from PyKDL import *
import numpy as np

def solve_ik_kdl(chain, target_pose, initial_guess=None):
    """
    Solve IK using KDL Newton-Raphson solver

    Args:
        chain: KDL Chain object
        target_pose: Desired end-effector pose (Frame)
        initial_guess: Initial joint angles (JntArray)

    Returns:
        (success, joint_angles)
    """
    # Create solvers
    fk_solver = ChainFkSolverPos_recursive(chain)
    ik_solver = ChainIkSolverPos_NR(
        chain,
        fk_solver,
        ChainIkSolverVel_pinv(chain)  # Velocity IK solver
    )

    # Initial guess (if not provided, use zeros)
    n_joints = chain.getNrOfJoints()
    if initial_guess is None:
        q_init = JntArray(n_joints)
        for i in range(n_joints):
            q_init[i] = 0.0
    else:
        q_init = initial_guess

    # Solve IK
    q_out = JntArray(n_joints)
    result = ik_solver.CartToJnt(q_init, target_pose, q_out)

    # Check if solution found
    success = (result >= 0)

    # Extract joint angles
    joint_angles = [q_out[i] for i in range(n_joints)]

    return success, joint_angles

# Usage example
robot_chain = create_simple_arm()

# Define target pose: position (0.8, 0.3, 0.0), no rotation
target_position = Vector(0.8, 0.3, 0.0)
target_rotation = Rotation.Identity()
target_frame = Frame(target_rotation, target_position)

# Solve IK
success, joint_angles = solve_ik_kdl(robot_chain, target_frame)

if success:
    print(f"✅ IK solution found:")
    for i, angle in enumerate(joint_angles):
        print(f"  Joint {i+1}: {np.degrees(angle):.2f}°")

    # Verify with FK
    fk_solver = ChainFkSolverPos_recursive(robot_chain)
    q = JntArray(len(joint_angles))
    for i, angle in enumerate(joint_angles):
        q[i] = angle

    result_frame = Frame()
    fk_solver.JntToCart(q, result_frame)

    achieved_pos = result_frame.p
    print(f"\nVerification (FK):")
    print(f"  Target:   ({target_position.x():.3f}, {target_position.y():.3f}, {target_position.z():.3f})")
    print(f"  Achieved: ({achieved_pos.x():.3f}, {achieved_pos.y():.3f}, {achieved_pos.z():.3f})")
else:
    print("❌ IK solution not found")
```

---

## Part 3: Advanced IK with TRAC-IK

TRAC-IK is faster and more reliable than KDL for complex robots.

### Step 1: Install TRAC-IK

```bash
sudo apt install ros-humble-trac-ik ros-humble-trac-ik-python
```

---

### Step 2: Use TRAC-IK in Python

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trac_ik_python.trac_ik import IK

class TracIKSolver(Node):
    def __init__(self):
        super().__init__('trac_ik_solver')

        # Initialize TRAC-IK
        # Args: base_link, tip_link, URDF (optional), timeout, epsilon, solve_type
        self.ik_solver = IK(
            "base_link",
            "end_effector_link",
            urdf_string=self.get_robot_description(),
            timeout=0.005,  # 5ms
            epsilon=1e-5,  # Precision
            solve_type="Speed"  # or "Distance" for better solutions
        )

    def get_robot_description(self):
        """Get URDF from parameter server"""
        # In real application, fetch from /robot_description parameter
        # For now, return simple URDF
        urdf = """<?xml version="1.0"?>
<robot name="simple_arm">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="end_effector_link"/>

  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="-3.14" upper="3.14" velocity="1.0"/>
  </joint>

  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="-3.14" upper="3.14" velocity="1.0"/>
  </joint>

  <joint name="joint3" type="revolute">
    <parent link="link2"/>
    <child link="end_effector_link"/>
    <origin xyz="0.3 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="-3.14" upper="3.14" velocity="1.0"/>
  </joint>
</robot>
"""
        return urdf

    def solve_ik(self, x, y, z, roll, pitch, yaw, seed_angles=None):
        """
        Solve IK using TRAC-IK

        Returns:
            Joint angles or None if no solution
        """
        solution = self.ik_solver.get_ik(
            seed_angles or [0.0] * 3,  # Initial guess
            x, y, z,  # Position
            roll, pitch, yaw  # Orientation (quaternion or RPY)
        )

        return solution

# Usage
def main():
    rclpy.init()
    solver = TracIKSolver()

    # Solve IK for target position
    solution = solver.solve_ik(
        x=0.7, y=0.2, z=0.1,
        roll=0.0, pitch=0.0, yaw=0.0
    )

    if solution:
        print(f"✅ TRAC-IK solution: {[f'{np.degrees(a):.2f}°' for a in solution]}")
    else:
        print("❌ No IK solution found")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Part 4: MoveIt Integration

### Step 1: Create MoveIt Config

```bash
# Generate MoveIt config for your robot
ros2 launch moveit_setup_assistant setup_assistant.launch.py

# Follow GUI to:
# 1. Load URDF
# 2. Define planning groups (arm, gripper)
# 3. Add end effectors
# 4. Generate config package
```

---

### Step 2: Use MoveIt for IK in Python

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest
from geometry_msgs.msg import PoseStamped

class MoveItIKClient(Node):
    def __init__(self):
        super().__init__('moveit_ik_client')

        # Create IK service client
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )

        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for IK service...')

    def compute_ik(self, target_pose: PoseStamped, group_name: str = "arm"):
        """Compute IK using MoveIt"""

        # Create request
        request = GetPositionIK.Request()
        request.ik_request.group_name = group_name
        request.ik_request.pose_stamped = target_pose
        request.ik_request.timeout.sec = 5

        # Call service
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        if response.error_code.val == response.error_code.SUCCESS:
            joint_state = response.solution.joint_state
            return joint_state.position
        else:
            self.get_logger().error(f'IK failed with error code: {response.error_code.val}')
            return None

# Usage
def main():
    rclpy.init()
    ik_client = MoveItIKClient()

    # Create target pose
    target = PoseStamped()
    target.header.frame_id = "base_link"
    target.pose.position.x = 0.5
    target.pose.position.y = 0.3
    target.pose.position.z = 0.4
    target.pose.orientation.w = 1.0  # No rotation

    # Compute IK
    joint_positions = ik_client.compute_ik(target, group_name="arm")

    if joint_positions:
        print(f"✅ Joint positions: {joint_positions}")
    else:
        print("❌ IK failed")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Summary

In this lesson, you learned:

- ✅ Forward kinematics with DH parameters
- ✅ Inverse kinematics with KDL and TRAC-IK
- ✅ MoveIt integration for motion planning
- ✅ Handling singularities and joint limits

**Key Takeaways**:
- **FK**: Easy (just matrix multiplication)
- **IK**: Hard (multiple solutions, singularities)
- **TRAC-IK**: Faster and more reliable than KDL
- **MoveIt**: Production-ready solution with collision avoidance

---

## Next Lesson

Continue to [Bipedal Locomotion →](./bipedal-locomotion.md)
